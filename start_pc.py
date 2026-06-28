import os
import signal
import subprocess
import sys
import time
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parent
NPM = shutil.which("npm.cmd") or shutil.which("npm") or "npm"


PROCESSES = [
    (
        "pc-backend",
        [sys.executable, str(ROOT / "pc_backend" / "pc_ota_backend.py")],
        {
            "HPVC_RUNTIME_BASE_URL": "http://192.168.219.104:8000",
        }
    ),
    (
        "react-dev",
        [NPM, "run", "dev", "--", "--host", "0.0.0.0"],
        {}
    )
]


def check_prerequisites():
    vite_path = ROOT / "node_modules" / ".bin" / "vite"
    backend_path = ROOT / "pc_backend" / "pc_ota_backend.py"

    if vite_path.exists():
        if backend_path.exists():
            return True

        print(f"[start_pc] missing PC backend: {backend_path}", flush=True)
        return False

    print("[start_pc] missing Node dependencies: node_modules/.bin/vite", flush=True)
    print("[start_pc] install with: npm.cmd install", flush=True)
    return False


def start_process(name, command, extra_env):
    env = os.environ.copy()
    env.update(extra_env)

    print(f"[start_pc] starting {name}: {' '.join(command)}", flush=True)
    return subprocess.Popen(command, cwd=ROOT, env=env)


def stop_processes(processes):
    for name, process in processes:
        if process.poll() is None:
            print(f"[start_pc] stopping {name}", flush=True)
            process.terminate()

    deadline = time.time() + 5

    for name, process in processes:
        if process.poll() is None:
            remaining = max(0.1, deadline - time.time())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                print(f"[start_pc] killing {name}", flush=True)
                process.kill()


def main():
    processes = []

    if not check_prerequisites():
        return 1

    def handle_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)

    try:
        for name, command, extra_env in PROCESSES:
            processes.append((name, start_process(name, command, extra_env)))
            time.sleep(0.8)

            code = processes[-1][1].poll()
            if code is not None:
                raise RuntimeError(f"{name} exited with code {code}")

        print()
        print("[start_pc] all services started", flush=True)
        print("[start_pc] React HMI: http://127.0.0.1:5173", flush=True)
        print("[start_pc] PC backend: http://127.0.0.1:8080", flush=True)
        print("[start_pc] HPVC runtime target: http://192.168.219.104:8000", flush=True)
        print("[start_pc] press Ctrl+C to stop", flush=True)

        while True:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(f"{name} exited with code {code}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[start_pc] shutdown requested", flush=True)
    except Exception as exc:
        print(f"\n[start_pc] error: {exc}", flush=True)
        return 1
    finally:
        stop_processes(processes)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
