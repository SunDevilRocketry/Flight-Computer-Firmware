#
# Emulator Integration Test Runner Script
#
import os
import sys
import subprocess
import time
from pathlib import Path
import shutil
import signal

cwd = os.getcwd()

class Emulator:

    def __init__(self, fcf_root):
        self.root_dir = fcf_root
        self.process = None
        if not self.env_check():
            raise KeyError("Environment incorrectly set up! Please set the SDEC_COMPORT and EMULATOR_COMPORT environment variables.")

    def env_check(self) -> bool:
        env_vars = ["SDEC_COMPORT", "EMULATOR_COMPORT"]
        for var in env_vars:
            if var not in os.environ:
                print(var + " does not exist.")
                return False
        return True

    def build(self) -> bool:
        build_dir = os.path.join(self.root_dir, "app", "rev2")
        result = subprocess.run(
            ["make", "clean"],
            cwd=build_dir,
            text=True,
        )
        result = subprocess.run(
            ["make", "-j4", "emulator-coverage"],
            cwd=build_dir,
            text=True,
        )
        return result.returncode == 0

    def generate_coverage(self) -> bool:
        build_dir = Path(os.path.join(self.root_dir, "app", "rev2")).resolve()
        if os.name == "nt":
            # gcovr must run through Cygwin since build used Cygwin gcc
            cygwin_path = build_dir.as_posix().replace("C:", "/cygdrive/c").replace("c:", "/cygdrive/c")
            result = subprocess.run(
                ["C:/cygwin64/bin/bash.exe", "--login", "-c", f"cd '{cygwin_path}' && make coverage"],
                text=True,
            )
        else:
            result = subprocess.run(
                ["make", "coverage"],
                cwd=build_dir,
                text=True,
            )
        os.chdir(cwd)
        return result.returncode == 0

    def start(self, fast_arm = False) -> subprocess.Popen:
        args = []
        if os.name == "nt":
            args.append("build/appa.exe")
        else:
            args.append("build/appa")
        args.append("--no-gui")
        if fast_arm:
            args.append("--fast-arm")
        kwargs = {}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        build_dir = os.path.join(self.root_dir, "app", "rev2")
        os.chdir(cwd)
        os.chdir(build_dir)
        proc = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            text=True,
            **kwargs
        )
        assert proc.stdin is not None
        proc.stdin.write(os.environ["EMULATOR_COMPORT"] + "\n")
        proc.stdin.flush()
        proc.stdin.close()
        os.chdir(cwd)
        self.process = proc

    # this is super annoying for no reason on Windows
    def stop(self):
        try:
            if self.process.poll() is None:
                if os.name == "nt":
                    self.process.send_signal(signal.CTRL_C_EVENT)
                else:
                    self.process.send_signal(signal.SIGINT)
                self.process.wait(timeout=10)
        except KeyboardInterrupt:
            if self.process.poll() is None:
                self.process.wait()
        except subprocess.TimeoutExpired:
            print("Emulator failed to exit cleanly, killing.")
            self.process.kill()
            self.process.wait()

    def copy_coverage(self, dir):
        shutil.copytree(os.path.join(self.root_dir, "app/rev2/build/coverage"),
                        os.path.join(dir, "coverage"), dirs_exist_ok=True)