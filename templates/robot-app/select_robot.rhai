let robot = variable::get("robot");
file::rename("robots/" + robot + ".ron", "copperconfig.ron");
file::delete("robots");
file::delete("select_robot.rhai");
