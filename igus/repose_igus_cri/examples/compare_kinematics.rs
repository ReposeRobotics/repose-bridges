//! Compare Repose FK with the Cartesian TCP reported in the same CRI STATUS.
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use cu29::units::si::length::millimeter;
use repose_igus_cri::CuCriCartesianPose;
use repose_igus_cri::CuCriStatus;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::IgusCriRxChannels;
use repose_kinematics::CuJointPosition;
use repose_kinematics::igus_rebel_chain;
use serde::Deserialize;

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum AbcConvention {
    // Rz(C) Ry(B) Rx(A): fixed-axis X, then Y, then Z.
    Xyz,
    // Rz(A) Ry(B) Rx(C): intrinsic Z, then Y, then X.
    Zyx,
}

#[derive(Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct FrameOffset {
    translation: [Length; 3],
    rpy: [Angle; 3],
}

impl FrameOffset {
    fn validate(&self) -> CuResult<()> {
        if self
            .translation
            .iter()
            .any(|v| !v.get::<meter>().is_finite())
            || self.rpy.iter().any(|v| !v.get::<radian>().is_finite())
        {
            return Err(CuError::from("Frame offsets must be finite"));
        }
        Ok(())
    }

    fn transform(&self) -> Transform3D<f64> {
        Transform3D::<f64>::from_translation_euler_xyz(self.translation, self.rpy)
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CuComparisonParams {
    // Deliberately required: do not silently assume an ABC convention.
    abc: AbcConvention,
    #[serde(default)]
    bridge: CuIgusCriParams,
    // Controller reference frame -> Repose model base.
    #[serde(default)]
    base: FrameOffset,
    // Repose model flange -> controller's configured TCP.
    #[serde(default)]
    tool: FrameOffset,
}

#[derive(Default)]
struct PoseError {
    position: Length,
    orientation: Angle,
}

fn controller_transform(
    pose: &CuCriCartesianPose,
    convention: AbcConvention,
    output: &mut Transform3D<f64>,
) {
    let rpy = match convention {
        AbcConvention::Xyz => pose.abc,
        AbcConvention::Zyx => [pose.abc[2], pose.abc[1], pose.abc[0]],
    };
    *output = Transform3D::<f64>::from_translation_euler_xyz(pose.translation, rpy);
}

fn compare_poses(
    predicted: &Transform3D<f64>,
    reference: &Transform3D<f64>,
    output: &mut PoseError,
) {
    let p = predicted.translation();
    let q = reference.translation();
    let squared_distance: f64 = p
        .iter()
        .zip(q)
        .map(|(p, q)| (p.get::<meter>() - q.get::<meter>()).powi(2))
        .sum();
    output.position = Length::new::<meter>(squared_distance.sqrt());
    let a = predicted.rotation();
    let b = reference.rotation();
    // trace(A^T B) is the Frobenius inner product. The resulting angle is
    // invariant to Euler wrapping and remains valid at Euler singularities.
    let trace: f64 = a
        .iter()
        .flatten()
        .zip(b.iter().flatten())
        .map(|(a, b)| a * b)
        .sum();
    output.orientation = Angle::new::<radian>(((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos());
}

fn main() -> CuResult<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| CuError::from("Usage: compare_kinematics <comparison.ron>"))?;
    let text = std::fs::read_to_string(path)
        .map_err(|e| CuError::new_with_cause("Cannot read comparison config", e))?;
    let params: CuComparisonParams = ron::from_str(&text)
        .map_err(|e| CuError::new_with_cause("Invalid comparison config", e))?;
    params.bridge.validate()?;
    params.base.validate()?;
    params.tool.validate()?;
    let base = params.base.transform();
    let tool = params.tool.transform();
    println!(
        "ABC convention: {:?}; comparison: controller reference → TCP",
        params.abc
    );
    let mut bridge = CuIgusCriBridge::from_params(params.bridge)?;
    let ctx = CuContext::new_with_clock();
    let chain = igus_rebel_chain().with_tool_offset(tool);
    let mut fk = Transform3D::<f64>::identity();
    let mut reference = Transform3D::<f64>::identity();
    let mut error = PoseError::default();
    let mut msg = CuMsg::<CuCriStatus>::default();
    let mut count = 0u32;
    let mut position_sum = 0.0f64;
    let mut position_max = 0.0f64;
    let mut orientation_sum = 0.0f64;
    let mut orientation_max = 0.0f64;
    bridge.start(&ctx)?;
    for _ in 0..1000 {
        bridge.receive(&ctx, &IgusCriRxChannels::CRI_STATUS, &mut msg)?;
        if let Some(status) = msg.payload() {
            let cartesian = status.cartesian.as_ref().ok_or_else(|| {
                CuError::from("CRI STATUS has no POSCARTROBOT; pose comparison unavailable")
            })?;
            let positions = status.joints.map(CuJointPosition::from);
            chain.forward_kinematics(&positions, &mut fk)?;
            let predicted = base.compose(fk);
            controller_transform(cartesian, params.abc, &mut reference);
            compare_poses(&predicted, &reference, &mut error);
            if count == 0 {
                let joints = status.joints.map(|value| value.get::<degree>());
                let xyz = cartesian.translation.map(|value| value.get::<millimeter>());
                let abc = cartesian.abc.map(|value| value.get::<degree>());
                let predicted_xyz = predicted
                    .translation()
                    .map(|value| value.get::<millimeter>());
                let delta = core::array::from_fn::<_, 3, _>(|i| predicted_xyz[i] - xyz[i]);
                println!("Snapshot joints A1…A6 (°): {joints:.3?}");
                println!("Controller XYZ (mm): {xyz:.3?} | ABC (°): {abc:.3?}");
                println!(
                    "Repose XYZ (mm): {predicted_xyz:.3?} | ΔXYZ Repose − controller (mm): {delta:.3?}"
                );
                println!("Repose rotation: {:.6?}", predicted.rotation());
                println!(
                    "Controller rotation ({:?}): {:.6?}",
                    params.abc,
                    reference.rotation()
                );
                let mut candidate = Transform3D::<f64>::identity();
                let mut candidate_error = PoseError::default();
                for convention in [AbcConvention::Xyz, AbcConvention::Zyx] {
                    controller_transform(cartesian, convention, &mut candidate);
                    compare_poses(&predicted, &candidate, &mut candidate_error);
                    println!(
                        "ABC hypothesis {convention:?} orientation error (°): {:.3}",
                        candidate_error.orientation.get::<degree>()
                    );
                }
            }
            let position = error.position.get::<millimeter>();
            let orientation = error.orientation.get::<degree>();
            println!("Position error: {position:.3} mm | Orientation error: {orientation:.3}°");
            position_sum += position;
            position_max = position_max.max(position);
            orientation_sum += orientation;
            orientation_max = orientation_max.max(orientation);
            count += 1;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    bridge.stop(&ctx)?;
    if count == 0 {
        return Err(CuError::from("No fresh CRI status observations received"));
    }
    println!(
        "Samples: {count} | Position mean/max: {:.3}/{position_max:.3} mm | Orientation mean/max: {:.3}/{orientation_max:.3}°",
        position_sum / f64::from(count),
        orientation_sum / f64::from(count)
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn angle(value: f64) -> Angle {
        Angle::new::<degree>(value)
    }

    #[test]
    fn test_known_translation_and_rotation_error() {
        let identity = Transform3D::<f64>::identity();
        let reference = Transform3D::<f64>::from_translation_euler_xyz(
            [
                Length::new::<millimeter>(3.0),
                Length::new::<millimeter>(4.0),
                Length::default(),
            ],
            [angle(0.0), angle(0.0), angle(90.0)],
        );
        let mut error = PoseError::default();
        compare_poses(&identity, &reference, &mut error);
        assert!((error.position.get::<millimeter>() - 5.0).abs() < 1e-9);
        assert!((error.orientation.get::<degree>() - 90.0).abs() < 1e-9);
        compare_poses(&reference, &reference, &mut error);
        assert!(error.position.get::<millimeter>() < 1e-9);
        assert!(error.orientation.get::<degree>() < 1e-5);
    }

    #[test]
    fn test_angle_wrapping_and_half_turn() {
        let mut error = PoseError::default();
        let positive = Transform3D::<f64>::from_translation_euler_xyz(
            [Length::default(); 3],
            [angle(0.0), angle(0.0), angle(179.0)],
        );
        let negative = Transform3D::<f64>::from_translation_euler_xyz(
            [Length::default(); 3],
            [angle(0.0), angle(0.0), angle(-179.0)],
        );
        compare_poses(&positive, &negative, &mut error);
        assert!((error.orientation.get::<degree>() - 2.0).abs() < 1e-9);
        let half_turn = Transform3D::<f64>::from_translation_euler_xyz(
            [Length::default(); 3],
            [angle(180.0), angle(0.0), angle(0.0)],
        );
        compare_poses(&half_turn, &Transform3D::<f64>::identity(), &mut error);
        assert!((error.orientation.get::<degree>() - 180.0).abs() < 1e-9);
    }

    #[test]
    fn test_frame_offsets_and_noncommuting_abc() {
        // Rz(90) Ry(90) Rx(0), independently known matrix.
        let pose = CuCriCartesianPose {
            translation: [Length::default(); 3],
            abc: [angle(0.0), angle(90.0), angle(90.0)],
        };
        let mut transform = Transform3D::<f64>::identity();
        controller_transform(&pose, AbcConvention::Xyz, &mut transform);
        let expected = [[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]];
        for (actual, expected) in transform
            .rotation()
            .iter()
            .flatten()
            .zip(expected.iter().flatten())
        {
            assert!((actual - expected).abs() < 1e-12);
        }
        // Base rotated 90° about Z maps a tool's local X offset into world Y.
        let base = FrameOffset {
            translation: [Length::default(); 3],
            rpy: [angle(0.0), angle(0.0), angle(90.0)],
        }
        .transform();
        let tool = FrameOffset {
            translation: [
                Length::new::<meter>(0.1),
                Length::default(),
                Length::default(),
            ],
            ..Default::default()
        }
        .transform();
        let composed = base.compose(Transform3D::<f64>::identity()).compose(tool);
        let xyz = composed.translation();
        assert!(xyz[0].get::<meter>().abs() < 1e-12);
        assert!((xyz[1].get::<meter>() - 0.1).abs() < 1e-12);
    }

    #[test]
    fn test_provisional_fhtw_alignment_on_recorded_snapshots() {
        let params: CuComparisonParams =
            ron::from_str(include_str!("compare_fhtw_provisional.ron")).unwrap();
        params.base.validate().unwrap();
        params.tool.validate().unwrap();
        let chain = igus_rebel_chain().with_tool_offset(params.tool.transform());
        // Recorded STATUS pairs. These exercise frame composition and units;
        // their similar orientations do not establish a general calibration.
        let snapshots = [
            (
                [-18.520, 4.130, 28.550, 6.330, 92.320, -70.950],
                [274.200, -76.900, 672.000],
                [-114.550, 18.880, -127.070],
            ),
            (
                [-20.040, 3.570, 95.690, 16.940, 26.890, -87.300],
                [396.400, -126.500, 371.300],
                [-114.550, 18.880, -127.090],
            ),
        ];
        for (joints, xyz, abc) in snapshots {
            let joints = joints.map(|v| CuJointPosition::from(angle(v)));
            let pose = CuCriCartesianPose {
                translation: xyz.map(Length::new::<millimeter>),
                abc: abc.map(angle),
            };
            let mut fk = Transform3D::<f64>::identity();
            chain.forward_kinematics(&joints, &mut fk).unwrap();
            let mut reference = Transform3D::<f64>::identity();
            controller_transform(&pose, params.abc, &mut reference);
            let mut error = PoseError::default();
            compare_poses(&params.base.transform().compose(fk), &reference, &mut error);
            assert!(error.position.get::<millimeter>() < 2.0);
            assert!(error.orientation.get::<degree>() < 0.03);
        }
    }

    #[test]
    fn test_abc_convention_and_config_validation() {
        let pose = CuCriCartesianPose {
            translation: [Length::default(); 3],
            abc: [angle(90.0), angle(0.0), angle(0.0)],
        };
        let mut output = Transform3D::<f64>::identity();
        controller_transform(&pose, AbcConvention::Xyz, &mut output);
        assert!((output.rotation()[2][1] - 1.0).abs() < 1e-12); // X rotation
        controller_transform(&pose, AbcConvention::Zyx, &mut output);
        assert!((output.rotation()[1][0] - 1.0).abs() < 1e-12); // Z rotation
        assert!(ron::from_str::<CuComparisonParams>("()").is_err());
        assert!(ron::from_str::<CuComparisonParams>("(abc: typo)").is_err());
        assert!(ron::from_str::<CuComparisonParams>("(abc: xyz, typo: 1)").is_err());
        let params: CuComparisonParams =
            ron::from_str(include_str!("compare_kinematics.ron")).unwrap();
        params.base.validate().unwrap();
        params.tool.validate().unwrap();
    }
}
