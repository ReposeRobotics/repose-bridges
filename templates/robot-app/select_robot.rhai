let robot = variable::get("robot");
file::rename("robots/" + robot + ".ron", "copperconfig.ron");
file::delete("robots");
if robot != "so_arm" {
    file::delete("sim");
    file::delete("src/sim.rs");
}
file::delete("select_robot.rhai");
