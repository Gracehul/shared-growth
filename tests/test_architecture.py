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


def test_stage2_has_no_hardware_imports() -> None:
    banned_roots = {"pymycobot", "serial"}
    banned_project_modules = {
        "drawing_robot.connection",
        "drawing_robot.robot.io",
        "drawing_robot.robot.service",
    }
    offenders = []
    for path in (ROOT / "drawing_robot" / "stage2").rglob("*.py"):
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
