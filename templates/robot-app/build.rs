use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Robot {
    IgusRebel,
    Ur5e,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Selection {
    robot: Robot,
}

fn main() {
    println!("cargo:rerun-if-changed=robot.ron");
    println!("cargo:rerun-if-changed=robots/igus_rebel.ron");
    println!("cargo:rerun-if-changed=robots/ur5e.ron");
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest directory"));
    let selection: Selection = ron::from_str(
        &fs::read_to_string(root.join("robot.ron")).expect("read robot.ron"),
    )
    .expect("robot.ron: choose robot: igus_rebel or robot: ur5e");
    let profile = match selection.robot {
        Robot::IgusRebel => "robots/igus_rebel.ron",
        Robot::Ur5e => "robots/ur5e.ron",
    };
    let graph = fs::read(root.join(profile)).expect("read robot profile");
    let destination = root.join("copperconfig.ron");
    if fs::read(&destination).ok().as_deref() != Some(graph.as_slice()) {
        fs::write(destination, graph).expect("generate copperconfig.ron");
    }
    cu29_build::setup();
}
