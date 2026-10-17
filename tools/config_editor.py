#!/usr/bin/env python3
"""Loopback-only configuration editor with an isolated local-simulation runner.

The HTTP API can edit/save/export profiles and launch tools/run_local_sim.py on
the current editor state. It never starts ROS, a chassis driver, or an armed
vehicle process. Simulation subprocesses are serialized and confined to
output/ and .runtime/ui-sim/.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import datetime
import errno
import http.client
import io
import json
import mimetypes
import os
import secrets
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
from profile_config import (ROOT, default_profile, validate, save_profile,
                            safe_name, MOTION_PRESETS, MOTION_LIMITS)

ARTIFACT_NAMES = ("video.mp4", "map.png", "summary.json", "perception.json",
                  "cones.json", "trajectory.csv")
SIM_TIMEOUT_S = 300.0
MAX_LOG_LINES = 400


def build_sim_command(profile_path, outdir, *, route="A", planner="native",
                      vision="mono", video=False, seed=3, max_time=180.0):
    """Build a fixed local-simulation command; never accepts a shell string."""
    route = str(route).upper()
    if route not in ("A", "B"):
        raise ValueError("route must be A or B")
    if planner not in ("native", "python", "auto"):
        raise ValueError("planner must be native, python or auto")
    if vision not in ("mono", "color", "card"):
        raise ValueError("vision must be mono, color or card")
    seed = int(seed)
    if seed < 0 or seed > 2**31 - 1:
        raise ValueError("seed out of range")
    cmd = [sys.executable, "-B", str(ROOT / "tools/run_local_sim.py"),
           "--profile", str(profile_path), "--route", route,
           "--seed", str(seed), "--outdir", str(outdir),
           "--planner-engine", planner, "--max-time", str(float(max_time))]
    if not video:
        cmd.append("--no-video")
    if vision == "color":
        cmd.append("--color")
    elif vision == "card":
        cmd += ["--use-card", "--card-color", "blue" if route == "A" else "red"]
    return cmd


def artifact_allowed(name):
    return isinstance(name, str) and name in ARTIFACT_NAMES


class SimulationManager:
    """Run one local simulation at a time; expose only local result artifacts."""

    def __init__(self):
        self.lock = threading.RLock()
        self.jobs = {}
        self.active = None
        self.last = None
        self.closed = False
        # The vehicle-only package intentionally ships without sim/.
        self.available = ((ROOT / "sim").is_dir()
                          and (ROOT / "tools/run_local_sim.py").is_file())

    # ------------------------------------------------------------------ #
    def start(self, profile, *, name="ui", route="A", planner="native",
              vision="mono", video=False, seed=3, max_time=180.0):
        if not self.available:
            raise ValueError("当前包不包含本机仿真（sim/ 与 tools/run_local_sim.py）；"
                             "请使用源码仓库或完整仿真包")
        result = validate(profile)
        if not result["valid"]:
            raise ValueError("; ".join(result["errors"]))
        route = str(route).upper()
        if route not in ("A", "B"):
            raise ValueError("路线必须是 A 或 B")
        if planner not in ("native", "python", "auto"):
            raise ValueError("规划引擎必须是 native、python 或 auto")
        if vision not in ("mono", "color", "card"):
            raise ValueError("视觉模式必须是 mono、color 或 card")
        try:
            seed = int(seed)
        except (TypeError, ValueError):
            raise ValueError("seed 必须是整数") from None
        if not 0 <= seed <= 2**31 - 1:
            raise ValueError("seed 超出范围")
        try:
            clean = safe_name(str(name or "ui"))
        except ValueError:
            clean = "ui"
        with self.lock:
            if self.closed:
                raise ValueError("配置界面正在关闭")
            active = self.jobs.get(self.active) if self.active else None
            if active and active["state"] in ("starting", "running", "cancelling"):
                raise ValueError("已有本机仿真在运行，请等待完成或先停止")
            job_id = secrets.token_hex(8)
            stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            outdir = ROOT / "output" / f"ui_{clean}_{route}_{stamp}"
            while outdir.exists():
                stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                outdir = ROOT / "output" / f"ui_{clean}_{route}_{stamp}"
            scene = ROOT / ".runtime/ui-sim" / job_id / "scene.json"
            scene.parent.mkdir(parents=True, exist_ok=True)
            scene.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n")
            cmd = build_sim_command(scene, outdir, route=route, planner=planner,
                                    vision=vision, video=bool(video), seed=seed,
                                    max_time=max_time)
            job = {"id": job_id, "state": "starting", "route": route,
                   "planner": planner, "vision": vision, "video": bool(video),
                   "seed": seed, "created": time.time(), "finished": None,
                   "outdir": str(outdir), "scene": str(scene), "cmd": cmd,
                   "log": [], "summary": None, "returncode": None,
                   "cancel_requested": False, "timed_out": False, "proc": None}
            self.jobs[job_id] = job
            self.active = job_id
            self.last = job_id
            threading.Thread(target=self._run, args=(job_id,), daemon=True).start()
            return self._status_locked(job)

    # ------------------------------------------------------------------ #
    def _append_log(self, job_id, line):
        with self.lock:
            job = self.jobs.get(job_id)
            if job is None:
                return
            job["log"].append(str(line).rstrip("\n"))
            if len(job["log"]) > MAX_LOG_LINES:
                del job["log"][:-MAX_LOG_LINES]

    def _run(self, job_id):
        with self.lock:
            job = self.jobs[job_id]
        timer = None
        code = None
        try:
            proc = subprocess.Popen(
                job["cmd"], cwd=str(ROOT), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                errors="replace", bufsize=1, start_new_session=True)
            with self.lock:
                job["proc"] = proc
                job["state"] = "running"
            timer = threading.Timer(SIM_TIMEOUT_S, self._timeout, args=(job_id,))
            timer.daemon = True
            timer.start()
            assert proc.stdout is not None
            for line in proc.stdout:
                self._append_log(job_id, line)
            code = proc.wait()
        except Exception as exc:  # noqa: BLE001 - report startup/runtime errors in UI
            self._append_log(job_id, f"[ui] 本机仿真启动失败：{exc}")
        finally:
            if timer is not None:
                timer.cancel()
        summary = None
        summary_path = Path(job["outdir"]) / "summary.json"
        if summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text())
            except (OSError, ValueError) as exc:
                self._append_log(job_id, f"[ui] summary.json 读取失败：{exc}")
        with self.lock:
            job["summary"] = summary
            job["returncode"] = code
            job["finished"] = time.time()
            job["proc"] = None
            if job.get("cancel_requested"):
                job["state"] = "cancelled"
            elif job.get("timed_out"):
                job["state"] = "timeout"
            elif code == 0 and summary and summary.get("success"):
                job["state"] = "success"
            elif code == 0 and summary:
                job["state"] = "failed"
            elif code == 0:
                job["state"] = "success"
            else:
                job["state"] = "failed"
            if self.active == job_id:
                self.active = None

    def _terminate(self, job_id, sig=signal.SIGTERM):
        with self.lock:
            job = self.jobs.get(job_id)
            proc = job.get("proc") if job else None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.terminate() if sig == signal.SIGTERM else proc.kill()
            except OSError:
                pass

    def _timeout(self, job_id):
        with self.lock:
            job = self.jobs.get(job_id)
            if job is None or job["state"] not in ("starting", "running"):
                return
            job["timed_out"] = True
        self._append_log(job_id, f"[ui] 超过 {int(SIM_TIMEOUT_S)}s，终止本机仿真。")
        self._terminate(job_id, signal.SIGTERM)
        killer = threading.Timer(3.0, self._terminate, args=(job_id, signal.SIGKILL))
        killer.daemon = True
        killer.start()

    def cancel(self, job_id=None):
        with self.lock:
            job = self.jobs.get(job_id) if job_id else self.jobs.get(self.active)
            if job is None or job["state"] not in ("starting", "running", "cancelling"):
                return self.status(job_id)
            job["cancel_requested"] = True
            job["state"] = "cancelling"
        self._terminate(job["id"], signal.SIGTERM)
        killer = threading.Timer(3.0, self._terminate, args=(job["id"], signal.SIGKILL))
        killer.daemon = True
        killer.start()
        return self.status(job["id"])

    def shutdown(self):
        with self.lock:
            self.closed = True
            active = self.active
        if active:
            self._terminate(active, signal.SIGTERM)

    # ------------------------------------------------------------------ #
    def _status_locked(self, job):
        outdir = Path(job["outdir"])
        summary = job.get("summary") or {}
        picked = {k: summary.get(k) for k in
                  ("success", "route", "expected_route", "final_state",
                   "reached_camp", "run_time_s", "wall_time_s",
                   "real_time_factor", "cone_contacts", "wall_contacts",
                   "red_cone_hits", "blue_cone_hits", "min_cone_clearance_m",
                   "perception")}
        return {
            "id": job["id"], "state": job["state"], "route": job["route"],
            "planner": job["planner"], "vision": job["vision"],
            "video": job["video"], "seed": job["seed"],
            "outdir": str(outdir), "elapsed_s": round(
                (job.get("finished") or time.time()) - job["created"], 2),
            "log": "\n".join(job["log"][-120:]),
            "summary": picked if summary else None,
            "artifacts": {name: (outdir / name).is_file() for name in ARTIFACT_NAMES},
        }

    def status(self, job_id=None):
        with self.lock:
            job = self.jobs.get(job_id) if job_id else self.jobs.get(self.active or self.last)
            if job is None:
                return {"state": "idle", "artifacts": {}, "log": "", "summary": None}
            return self._status_locked(job)

    def artifact(self, job_id, name):
        if not artifact_allowed(name):
            return None
        with self.lock:
            job = self.jobs.get(job_id)
            if job is None:
                return None
            root = Path(job["outdir"]).resolve()
        path = (root / name).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            return None
        return path


def existing_editor(port):
    """Only reuse an editor that identifies this exact project, without proxying."""
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=.5)
    try:
        connection.request("GET", "/health")
        response = connection.getresponse()
        if response.status != 200:
            return False
        data = json.loads(response.read(4096))
        return isinstance(data, dict) and data.get("service") == "crossfire-config-editor" and data.get("project") == str(ROOT.resolve())
    except (OSError, ValueError, http.client.HTTPException):
        return False
    finally:
        connection.close()


def listen_or_reuse(handler, port=None):
    automatic = port is None
    ports = list(range(8765, 8785)) + [0] if automatic else [port]
    if automatic:
        # Reuse a fallback instance even if the original default port became free.
        for candidate in ports[:-1]:
            if existing_editor(candidate):
                return None, candidate
    for candidate in ports:
        try:
            server = ThreadingHTTPServer(("127.0.0.1", candidate), handler)
            return server, server.server_port
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
            if existing_editor(candidate):
                return None, candidate
            if not automatic:
                raise ValueError(f"端口 {candidate} 已被其他服务或旧版界面占用。可运行 bash tools/configure.sh 自动选端口，或加 --port 0 使用空闲端口。") from None
    raise RuntimeError("无法分配本地端口")


def main():
    ap = argparse.ArgumentParser(description="本机配置界面；默认复用同项目实例或自动选择空闲端口")
    ap.add_argument("--port", type=int, help="指定端口；0表示自动分配空闲端口")
    a = ap.parse_args()
    if a.port is not None and not 0 <= a.port <= 65535:
        ap.error("端口应在0～65535之间")
    token = secrets.token_urlsafe(24)
    manager = SimulationManager()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, value, status=200, kind="application/json; charset=utf-8"):
            data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()
            self.wfile.write(data)

        def respond_file(self, path):
            """Serve a local result artifact with minimal HTTP Range support."""
            size = path.stat().st_size
            start, end, status = 0, size - 1, 200
            range_header = self.headers.get("Range", "")
            if range_header.startswith("bytes=") and size > 0:
                try:
                    spec = range_header[6:].split(",", 1)[0]
                    left, right = spec.split("-", 1)
                    if left:
                        start = int(left)
                    if right:
                        end = min(int(right), size - 1)
                    if start < 0 or start > end or start >= size:
                        raise ValueError
                    status = 206
                except ValueError:
                    start, end, status = 0, size - 1, 200
            length = max(0, end - start + 1)
            self.send_response(status)
            self.send_header("Content-Type", mimetypes.guess_type(path.name)[0] or "application/octet-stream")
            self.send_header("Content-Length", str(length))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if status == 206:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            with path.open("rb") as handle:
                handle.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = handle.read(min(65536, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)

        def authorized(self, parsed=None):
            host = self.headers.get("Host", "").split(":")[0]
            supplied = self.headers.get("X-Config-Token")
            if not supplied and parsed is not None:
                supplied = urllib.parse.parse_qs(parsed.query).get("token", [""])[0]
            return (host in ("127.0.0.1", "localhost") and supplied
                    and secrets.compare_digest(supplied, token))

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)
            if self.headers.get("Host", "").split(":")[0] not in ("localhost", "127.0.0.1"):
                return self.respond({"error": "host rejected"}, 403)
            if parsed.path == "/health":
                return self.respond(dict(service="crossfire-config-editor", project=str(ROOT.resolve())))
            if parsed.path == "/":
                html = (ROOT / "ui/config_editor.html").read_text().replace("__TOKEN__", token)
                return self.respond(html.encode(), kind="text/html; charset=utf-8")
            if not self.authorized(parsed):
                return self.respond({"error": "token required"}, 403)
            try:
                query = urllib.parse.parse_qs(parsed.query)
                name = query.get("name", ["default"])[0]
                if parsed.path == "/api/init":
                    profiles = sorted(p.stem for p in (ROOT / "profiles").glob("*.json"))
                    return self.respond(dict(
                        profile=default_profile(), presets=MOTION_PRESETS,
                        limits=MOTION_LIMITS, profiles=profiles,
                        hardware=json.loads((ROOT / "config/vehicle_profile.json").read_text()),
                        local_sim=manager.available,
                        test_defaults=dict(route="A", planner="native", vision="mono",
                                           video=False, seed=3)))
                if parsed.path == "/api/ui/status":
                    return self.respond(manager.status(query.get("job", [None])[0]))
                if parsed.path == "/api/ui/artifact":
                    path = manager.artifact(query.get("job", [""])[0],
                                            query.get("name", [""])[0])
                    if path is None:
                        return self.respond({"error": "artifact not found"}, 404)
                    return self.respond_file(path)
                safe_name(name)
                path = ROOT / "profiles" / (name + ".json")
                if parsed.path == "/api/load":
                    return self.respond(json.loads(path.read_text()))
                if parsed.path == "/api/export":
                    p = json.loads(path.read_text())
                    save_profile(name, p)
                    folder = ROOT / "profiles/generated" / name
                    buf = io.BytesIO()
                    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
                        for f in sorted(folder.iterdir()):
                            if f.is_file():
                                z.write(f, f.name)
                    return self.respond(buf.getvalue(), kind="application/zip")
                return self.respond({"error": "not found"}, 404)
            except (ValueError, OSError, KeyError) as e:
                return self.respond({"error": str(e)}, 400)

        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            if not self.authorized(parsed):
                return self.respond({"error": "token required"}, 403)
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 1000000:
                    raise ValueError("请求体应小于1MB")
                body = json.loads(self.rfile.read(length))
                if parsed.path == "/api/ui/cancel":
                    return self.respond(manager.cancel(body.get("job")))
                if parsed.path == "/api/ui/start":
                    return self.respond(manager.start(
                        body["profile"], name=body.get("name", "ui"),
                        route=body.get("route", "A"),
                        planner=body.get("planner", "native"),
                        vision=body.get("vision", "mono"),
                        video=bool(body.get("video", False)),
                        seed=body.get("seed", 3),
                        max_time=body.get("max_time", 180)))
                profile = body["profile"]
                if parsed.path == "/api/validate":
                    return self.respond(validate(profile))
                if parsed.path == "/api/save":
                    return self.respond(save_profile(body["name"], profile))
                return self.respond({"error": "not found"}, 404)
            except (ValueError, OSError, KeyError, TypeError, RuntimeError) as e:
                return self.respond({"error": str(e)}, 400)

    try:
        server, port = listen_or_reuse(Handler, a.port)
    except (OSError, ValueError) as error:
        manager.shutdown()
        ap.exit(2, f"配置界面启动失败：{error}\n")
    if server is None:
        print(f"本项目的配置界面已在运行，直接打开：http://127.0.0.1:{port}/\n已复用现有进程。关闭页面不会退出服务；在原启动终端按 Ctrl+C 可退出。", flush=True)
        return
    if a.port is None and port != 8765:
        print("默认端口已被占用，已选择空闲端口；原有进程保持运行。", flush=True)
    sim_note = ("可编辑/校验/导出配置，也可在页面内运行隔离的本机算法仿真；"
                if manager.available else
                "可编辑/校验/导出配置；当前包不含本机仿真（sim/）；")
    print(f"配置界面：http://127.0.0.1:{server.server_port}/\n"
          f"{sim_note}不连接 ROS 或底盘。Ctrl+C 退出。", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
