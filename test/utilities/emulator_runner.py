#
# Emulator Integration Test Runner Script
#
import os
import sys
import subprocess
import time
from pathlib import Path
import shutil

root_dir = sys.argv[1]
sdec_suite = sys.argv[2]
sdec_modules = sys.argv[3]

cwd = os.getcwd()

EMULATOR_STARTUP_DELAY = 3
INTERMEDIATE_RESULTS_DIR = "intermediate_results"


def env_check() -> bool:
    env_vars = ["SDEC_BASE", "SDEC_COMPORT", "EMULATOR_COMPORT", "SDEC_PYTHON"]
    for var in env_vars:
        if var not in os.environ:
            print(var + " does not exist.")
            return False
    return True

def build_emulator() -> bool:
    build_dir = os.path.join(root_dir, "app", "rev2")
    result = subprocess.run(
        ["make", "-j4", "emulator"],
        cwd=build_dir,
        text=True,
    )
    return result.returncode == 0

def run_emulator(fast_arm = False) -> subprocess.Popen:
    args = []
    if os.name == "nt":
        args.append("build/appa.exe")
    else:
        args.append("build/appa")
    args.append("--no-gui")
    if fast_arm:
        args.append("--fast-arm")
    build_dir = os.path.join(root_dir, "app", "rev2")
    os.chdir(cwd)
    os.chdir(build_dir)
    proc = subprocess.Popen(
        args,
        stdin=subprocess.PIPE,
        text=True,
    )
    assert proc.stdin is not None
    proc.stdin.write(os.environ["EMULATOR_COMPORT"] + "\n")
    proc.stdin.flush()
    proc.stdin.close()
    os.chdir(cwd)
    return proc

def _run_sdec_script(name: str) -> bool:
    os.chdir(os.environ["SDEC_BASE"])
    script = sdec_modules + "." + name
    py_inst = os.environ["SDEC_PYTHON"]
    result = subprocess.run([py_inst, "-m", script], text=True)
    return result.returncode == 0

def run_setup()   -> bool: return _run_sdec_script("setup")
def run_execute() -> bool: return _run_sdec_script("execute")
def run_verify()  -> bool: return _run_sdec_script("verify")

# START HERE
if not env_check():
    print("Your environment is not correctly configured! Please set the relevant environment variables.")
    exit(1)

# Build the binary
build_emulator()

# Setup Phase
emulator = run_emulator()
time.sleep(EMULATOR_STARTUP_DELAY)
run_setup()
if emulator.poll() is None:
    emulator.terminate()
    emulator.wait()

# Execute Phase
emulator = run_emulator(fast_arm=True)
time.sleep(EMULATOR_STARTUP_DELAY)
run_execute()
if emulator.poll() is None:
    emulator.terminate()
    emulator.wait()

# Verify Phase
emulator = run_emulator()
time.sleep(EMULATOR_STARTUP_DELAY)
run_verify()
if emulator.poll() is None:
    emulator.terminate()
    emulator.wait()

# Copy Results
shutil.copytree(os.path.join(os.getenv("SDEC_BASE"), sdec_suite, INTERMEDIATE_RESULTS_DIR),
                os.path.join(cwd, INTERMEDIATE_RESULTS_DIR), dirs_exist_ok=True)

