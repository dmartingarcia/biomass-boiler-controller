# PlatformIO pre-build script: embeds the git commit in the firmware as GIT_VERSION
# ("377c33b", or "377c33b-dirty" when the working tree has uncommitted changes).
Import("env")
import subprocess

def git(*args):
    try:
        return subprocess.check_output(["git", *args], cwd=env["PROJECT_DIR"], stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""

sha = git("rev-parse", "--short", "HEAD") or "unknown"
if git("status", "--porcelain"):
    sha += "-dirty"
env.Append(CPPDEFINES=[("GIT_VERSION", env.StringifyMacro(sha))])
print("Firmware version: " + sha)
