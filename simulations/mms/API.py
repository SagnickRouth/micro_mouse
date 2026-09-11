\"\"\"
MMS (Mackorone's Micromouse Simulator) API wrapper.

Communication protocol:
  - Commands are sent via stdout (print)
  - Responses are read via stdin (input)
  - Debug logging goes to stderr

Repo: https://github.com/mackorone/mms
\"\"\"

import sys


def log(msg: str) -> None:
    \"\"\"Log a debug message (appears in simulator console).\"\"\"
    sys.stderr.write(f"{msg}\n")
    sys.stderr.flush()


def _command(cmd: str) -> str:
    \"\"\"Send a command and return the response.\"\"\"
    print(cmd, flush=True)
    return sys.stdin.readline().strip()


def _command_no_response(cmd: str) -> None:
    \"\"\"Send a command that has no response.\"\"\"
    print(cmd, flush=True)


# ── Maze Info ──────────────────────────────────────────────
def mazeWidth() -> int:
    return int(_command("mazeWidth"))


def mazeHeight() -> int:
    return int(_command("mazeHeight"))


# ── Wall Queries ───────────────────────────────────────────
def wallFront() -> bool:
    return _command("wallFront") == "true"


def wallRight() -> bool:
    return _command("wallRight") == "true"


def wallLeft() -> bool:
    return _command("wallLeft") == "true"


# ── Movement ──────────────────────────────────────────────
def moveForward(n: int = 1) -> None:
    for _ in range(n):
        resp = _command("moveForward")
        if resp == "crash":
            log("CRASH!")
            return


def turnRight() -> None:
    _command_no_response("turnRight")


def turnLeft() -> None:
    _command_no_response("turnLeft")


def turnAround() -> None:
    \"\"\"180° turn (two right turns).\"\"\"
    turnRight()
    turnRight()


# ── Cell Visualization ────────────────────────────────────
def setWall(x: int, y: int, direction: str) -> None:
    \"\"\"Display a wall. direction = 'n', 'e', 's', 'w'.\"\"\"
    _command_no_response(f"setWall {x} {y} {direction}")


def clearWall(x: int, y: int, direction: str) -> None:
    _command_no_response(f"clearWall {x} {y} {direction}")


def setColor(x: int, y: int, color: str) -> None:
    \"\"\"Set cell color. Colors: r, g, b, c, m, y, w, o, a (gray), etc.\"\"\"
    _command_no_response(f"setColor {x} {y} {color}")


def clearColor(x: int, y: int) -> None:
    _command_no_response(f"clearColor {x} {y}")


def clearAllColor() -> None:
    _command_no_response("clearAllColor")


def setText(x: int, y: int, text: str) -> None:
    \"\"\"Display text in a cell (max ~10 chars).\"\"\"
    _command_no_response(f"setText {x} {y} {text}")


def clearText(x: int, y: int) -> None:
    _command_no_response(f"clearText {x} {y}")


def clearAllText() -> None:
    _command_no_response("clearAllText")


# ── Reset Detection ───────────────────────────────────────
def wasReset() -> bool:
    return _command("wasReset") == "true"


def ackReset() -> None:
    _command_no_response("ackReset")
