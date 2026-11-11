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
# Only what ends up in the firmware / web image counts (the IDE rewrites .vscode on its own)
if git("status", "--porcelain", "--", "src", "include", "data", "lib", "scripts", "platformio.ini"):
    sha += "-dirty"
env.Append(CPPDEFINES=[("GIT_VERSION", env.StringifyMacro(sha))])
print("Firmware version: " + sha)
