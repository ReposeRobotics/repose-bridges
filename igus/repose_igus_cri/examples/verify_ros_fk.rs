//! Verify Repose against timestamp-matched FK from an isolated ROS publisher.
//! Reference generation: `ros_reference/capture.py`. No robot connection is opened.
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuKinematicChain;
use repose_kinematics::igus_rebel_chain;
use repose_robot_description::CuRobotDescription;
use repose_robot_description::CuUrdfImport;
use serde::Deserialize;
use serde::Serialize;
use sha2::Digest;
use sha2::Sha256;
use std::path::Path;

// Numerical implementation tolerances, not robot accuracy specifications.
const POSITION_TOLERANCE: f64 = 1e-9; // metres
const ROTATION_TOLERANCE: f64 = 1e-7; // radians

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    schema: u32,
    source: String,
    reference: String,
    capture_utc: String,
    publisher_sha256: String,
    urdf_sha256: String,
    kinematic_urdf_sha256: String,
    base_frame: String,
    tip_frame: String,
    joint_names: [String; 6],
    samples: Vec<Sample>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Sample {
    name: String,
    // JSON fixture boundary uses documented SI values; FK receives typed angles.
    joints: [f64; 6],
    transform: [[f64; 4]; 4],
    stamp: [i64; 2],
}

#[derive(Serialize)]
struct ModelResult {
    model: &'static str,
    matches: bool,
    samples: Vec<SampleResult>,
}

#[derive(Serialize)]
struct SampleResult {
    name: String,
    position_error_m: f64,
    orientation_error_rad: f64,
}

fn verify_hash(path: &Path, expected: &str) -> CuResult<String> {
    let bytes =
        std::fs::read(path).map_err(|e| CuError::new_with_cause("Cannot read captured URDF", e))?;
    let actual = format!("{:x}", Sha256::digest(&bytes));
    if actual != expected {
        return Err(CuError::from("Captured URDF SHA-256 mismatch"));
    }
    String::from_utf8(bytes).map_err(|e| CuError::new_with_cause("URDF is not UTF-8", e))
}

fn validate_reference(reference: &Reference) -> CuResult<()> {
    if reference.schema != 1
        || reference.reference != "ROS robot_state_publisher"
        || reference.base_frame != "base_link"
        || !["link6", "tool0", "TCPGripper"].contains(&reference.tip_frame.as_str())
        || reference.samples.is_empty()
        || reference.joint_names != core::array::from_fn::<_, 6, _>(|i| format!("joint{}", i + 1))
    {
        return Err(CuError::from(
            "Expected nonempty named ReBeL frame reference with joints 1…6",
        ));
    }
    for sample in &reference.samples {
        if sample.name.is_empty()
            || sample.joints.iter().any(|v| !v.is_finite())
            || sample.transform.iter().flatten().any(|v| !v.is_finite())
            || sample.stamp[0] <= 0
            || !(0..1_000_000_000).contains(&sample.stamp[1])
            || sample.transform[3] != [0.0, 0.0, 0.0, 1.0]
        {
            return Err(CuError::from("Invalid or unstamped reference sample"));
        }
        let r = &sample.transform;
        for i in 0..3 {
            for j in 0..3 {
                let product: f64 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
                let expected = if i == j { 1.0 } else { 0.0 };
                if (product - expected).abs() > 1e-10 {
                    return Err(CuError::from("Reference rotation is not orthonormal"));
                }
            }
        }
        let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        if (det - 1.0).abs() > 1e-10 {
            return Err(CuError::from("Reference rotation has invalid determinant"));
        }
    }
    Ok(())
}

fn compare(
    chain: &CuKinematicChain<6>,
    reference: &Reference,
    output: &mut ModelResult,
) -> CuResult<()> {
    let mut pose = Transform3D::<f64>::identity();
    output.matches = true;
    output.samples.clear();
    for sample in &reference.samples {
        let joints = sample
            .joints
            .map(|v| CuJointPosition::from(Angle::new::<radian>(v)));
        chain.forward_kinematics(&joints, &mut pose)?;
        let matrix = pose.to_matrix();
        let position: f64 = (0..3)
            .map(|i| (matrix[i][3] - sample.transform[i][3]).powi(2))
            .sum::<f64>()
            .sqrt();
        // Relative rotation A^T B; atan2 is stable near identity and pi.
        let mut relative = [[0.0; 3]; 3];
        for (i, row) in relative.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (0..3).map(|k| matrix[k][i] * sample.transform[k][j]).sum();
            }
        }
        let sine = ((relative[2][1] - relative[1][2]).powi(2)
            + (relative[0][2] - relative[2][0]).powi(2)
            + (relative[1][0] - relative[0][1]).powi(2))
        .sqrt()
            * 0.5;
        let cosine = (relative[0][0] + relative[1][1] + relative[2][2] - 1.0) * 0.5;
        let orientation = sine.atan2(cosine);
        output.matches &= position <= POSITION_TOLERANCE && orientation <= ROTATION_TOLERANCE;
        output.samples.push(SampleResult {
            name: sample.name.clone(),
            position_error_m: position,
            orientation_error_rad: orientation,
        });
    }
    Ok(())
}

fn main() -> CuResult<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| CuError::from("Usage: verify_ros_fk <reference.json>"))?;
    let path = Path::new(&path);
    let text = std::fs::read_to_string(path)
        .map_err(|e| CuError::new_with_cause("Cannot read ROS reference", e))?;
    let reference: Reference = serde_json::from_str(&text)
        .map_err(|e| CuError::new_with_cause("Invalid ROS reference JSON", e))?;
    validate_reference(&reference)?;
    let directory = path
        .parent()
        .ok_or_else(|| CuError::from("Missing reference directory"))?;
    verify_hash(&directory.join("robot.urdf"), &reference.urdf_sha256)?;
    let xml = verify_hash(
        &directory.join("robot.kinematics.urdf"),
        &reference.kinematic_urdf_sha256,
    )?;
    let imported = CuRobotDescription::<6, 0>::from_urdf(
        &xml,
        CuUrdfImport::new(&reference.base_frame, &reference.tip_frame),
    )?;
    if imported.joint_names() != &reference.joint_names {
        return Err(CuError::from(
            "Imported joint order differs from ROS inputs",
        ));
    }
    println!(
        "Reference: {} | {} | {}",
        reference.reference, reference.source, reference.capture_utc
    );
    println!(
        "URDF SHA-256: {} | Publisher SHA-256: {}",
        reference.urdf_sha256, reference.publisher_sha256
    );
    println!(
        "Frames: {} → {} | Samples: {}",
        reference.base_frame,
        reference.tip_frame,
        reference.samples.len()
    );
    println!("Numerical tolerances: {POSITION_TOLERANCE} m, {ROTATION_TOLERANCE} rad");
    let mut results = vec![ModelResult {
        model: "captured_urdf",
        matches: false,
        samples: Vec::new(),
    }];
    compare(imported.chain(), &reference, &mut results[0])?;
    if reference.tip_frame == "link6" {
        let mut preset = ModelResult {
            model: "igus_rebel_preset",
            matches: false,
            samples: Vec::new(),
        };
        compare(&igus_rebel_chain(), &reference, &mut preset)?;
        results.push(preset);
    }
    for result in &results {
        let position = result
            .samples
            .iter()
            .map(|s| s.position_error_m)
            .fold(0.0f64, f64::max);
        let orientation = result
            .samples
            .iter()
            .map(|s| s.orientation_error_rad)
            .fold(0.0f64, f64::max);
        println!(
            "{}: {} | max position: {position:.9e} m | max orientation: {orientation:.9e} rad",
            result.model,
            if result.matches { "MATCH" } else { "DIFF" }
        );
    }
    let json = serde_json::to_string_pretty(&results)
        .map_err(|e| CuError::new_with_cause("Cannot encode verification report", e))?;
    std::fs::write(directory.join("comparison.json"), json)
        .map_err(|e| CuError::new_with_cause("Cannot save verification report", e))?;
    if !results[0].matches {
        return Err(CuError::from(
            "Repose FK from captured URDF disagrees with ROS",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference() -> Reference {
        // Independent zero-angle model pose: base yaw pi and summed link lengths.
        Reference {
            schema: 1,
            source: "test".into(),
            reference: "ROS robot_state_publisher".into(),
            capture_utc: "test".into(),
            publisher_sha256: "test".into(),
            urdf_sha256: "test".into(),
            kinematic_urdf_sha256: "test".into(),
            base_frame: "base_link".into(),
            tip_frame: "link6".into(),
            joint_names: core::array::from_fn(|i| format!("joint{}", i + 1)),
            samples: vec![Sample {
                name: "zero".into(),
                joints: [0.0; 6],
                stamp: [1, 0],
                transform: [
                    [-1.0, 0.0, 0.0, -0.0013449814287804],
                    [0.0, -1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.869999407401356],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            }],
        }
    }

    #[test]
    fn test_comparison_detects_position_and_rotation_mismatch() {
        let mut reference = reference();
        validate_reference(&reference).unwrap();
        let mut result = ModelResult {
            model: "test",
            matches: false,
            samples: Vec::new(),
        };
        compare(&igus_rebel_chain(), &reference, &mut result).unwrap();
        assert!(result.matches);
        reference.samples[0].transform[0][3] += 0.01;
        compare(&igus_rebel_chain(), &reference, &mut result).unwrap();
        assert!(!result.matches);
        assert!((result.samples[0].position_error_m - 0.01).abs() < 1e-12);
        reference.samples[0].transform[0][3] -= 0.01;
        reference.samples[0].transform[0][0] = 1.0;
        reference.samples[0].transform[1][1] = 1.0;
        compare(&igus_rebel_chain(), &reference, &mut result).unwrap();
        assert!(!result.matches);
        assert!((result.samples[0].orientation_error_rad - core::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn test_reference_rejects_missing_nonfinite_and_invalid_rotation() {
        let mut value = reference();
        value.samples.clear();
        assert!(validate_reference(&value).is_err());
        let mut value = reference();
        value.joint_names.swap(0, 1);
        assert!(validate_reference(&value).is_err());
        let mut value = reference();
        value.samples[0].joints[0] = f64::NAN;
        assert!(validate_reference(&value).is_err());
        let mut value = reference();
        value.samples[0].transform[0][0] = 0.0;
        assert!(validate_reference(&value).is_err());
        let mut value = reference();
        value.samples[0].transform[0][0] = 1.0; // Reflection, not a proper rotation.
        assert!(validate_reference(&value).is_err());
        let mut value = reference();
        value.samples[0].stamp = [0, 0];
        assert!(validate_reference(&value).is_err());
    }
}
