"""
MMS (Mackorone's Micromouse Simulator) API wrapper.

Communication protocol:
  - Commands are sent via stdout (print)
  - Responses are read via stdin (input)
  - Debug logging goes to stderr

Repo: https://github.com/mackorone/mms
"""

import sys


def log(msg):
    """Log a debug message (appears in simulator console)."""
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()


def _command(cmd):
    """Send a command and return the response."""
    print(cmd, flush=True)
    return sys.stdin.readline().strip()


def _command_no_response(cmd):
    """Send a command that has no response."""
    print(cmd, flush=True)


# Maze Info
def mazeWidth():
    return int(_command("mazeWidth"))


def mazeHeight():
    return int(_command("mazeHeight"))


# Wall Queries
def wallFront():
    return _command("wallFront") == "true"


def wallRight():
    return _command("wallRight") == "true"


def wallLeft():
    return _command("wallLeft") == "true"


# Movement
def moveForward(n=1):
    for _ in range(n):
        resp = _command("moveForward")
        if resp == "crash":
            log("CRASH!")
            return


def turnRight():
    _command_no_response("turnRight")


def turnLeft():
    _command_no_response("turnLeft")


def turnAround():
    """180 degree turn."""
    turnRight()
    turnRight()


# Cell Visualization
def setWall(x, y, direction):
    _command_no_response("setWall {} {} {}".format(x, y, direction))


def clearWall(x, y, direction):
    _command_no_response("clearWall {} {} {}".format(x, y, direction))


def setColor(x, y, color):
    _command_no_response("setColor {} {} {}".format(x, y, color))


def clearColor(x, y):
    _command_no_response("clearColor {} {}".format(x, y))


def clearAllColor():
    _command_no_response("clearAllColor")


def setText(x, y, text):
    _command_no_response("setText {} {} {}".format(x, y, text))


def clearText(x, y):
    _command_no_response("clearText {} {}".format(x, y))


def clearAllText():
    _command_no_response("clearAllText")


# Reset Detection
def wasReset():
    return _command("wasReset") == "true"


def ackReset():
    _command_no_response("ackReset")
