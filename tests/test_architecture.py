import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent


def active_python_files():
    yield from (ROOT / "drawing_robot").rglob("*.py")
    yield from (ROOT / "scripts").glob("*.py")


def test_only_robot_io_imports_pymycobot() -> None:
    offenders = []
    allowed = ROOT / "drawing_robot" / "robot" / "io.py"
    for path in active_python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imports_pymycobot = any(
            (isinstance(node, ast.ImportFrom) and node.module == "pymycobot")
            or (
                isinstance(node, ast.Import)
                and any(alias.name == "pymycobot" for alias in node.names)
            )
            for node in ast.walk(tree)
        )
        if imports_pymycobot and path != allowed:
            offenders.append(path.relative_to(ROOT).as_posix())
    assert offenders == []


def test_only_robot_io_constructs_mycobot280() -> None:
    offenders = []
    allowed = ROOT / "drawing_robot" / "robot" / "io.py"
    for path in active_python_files():
        text = path.read_text(encoding="utf-8")
        if "MyCobot280(" in text and path != allowed:
            offenders.append(path.relative_to(ROOT).as_posix())
    assert offenders == []


def test_stage2_has_no_hardware_or_simulation_imports() -> None:
    banned_roots = {
        "pymycobot",
        "serial",
        "pybullet",
        "mujoco",
        "rclpy",
        "moveit",
        "gazebo",
    }
    banned_project_modules = {
        "drawing_robot.connection",
        "drawing_robot.robot.io",
        "drawing_robot.robot.service",
    }
    offenders = []
    paths = list((ROOT / "drawing_robot" / "stage2").rglob("*.py"))
    paths.append(ROOT / "scripts" / "visualize_trajectory.py")
    for path in paths:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            if any(
                name.split(".")[0] in banned_roots or name in banned_project_modules
                for name in names
            ):
                offenders.append(path.relative_to(ROOT).as_posix())
    assert offenders == []


def test_stage3_has_no_hardware_or_physics_imports() -> None:
    banned_roots = {
        "pymycobot",
        "serial",
        "pybullet",
        "mujoco",
        "rclpy",
        "moveit",
        "gazebo",
    }
    paths = list((ROOT / "drawing_robot" / "execution").rglob("*.py"))
    paths.extend(
        [ROOT / "scripts" / "run_simulation.py", ROOT / "scripts" / "replay_simulation.py"]
    )
    offenders = []
    for path in paths:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            if any(name.split(".")[0] in banned_roots for name in names):
                offenders.append(path.relative_to(ROOT).as_posix())
    assert offenders == []


def test_stage3_executor_has_no_simrobot_or_wall_clock_dependency() -> None:
    path = ROOT / "drawing_robot" / "execution" / "executor.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    direct_imports = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    assert not any("sim_robot" in module for module in imported)
    assert not ({"time", "datetime"} & direct_imports)
