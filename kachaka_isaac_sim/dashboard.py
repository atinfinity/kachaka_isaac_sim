"""Web dashboard: the map with the robot, shelves and charger, the robot's
state, and buttons that send commands, all through kachaka-api.

It only talks gRPC, so it runs as its own process next to the simulator (or
a real robot) and needs no ROS. scripts/dashboard.py starts it.

The handlers take a kachaka-api client; the conversions below take plain
attribute access so the unit tests can feed them stand-ins.
"""

import json
import math
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PAGE = Path(__file__).with_name("dashboard.html")

COMMAND_STATES = {0: "unspecified", 1: "pending", 2: "running"}
POWER_SUPPLY = {0: "unknown", 1: "charging", 2: "discharging", 3: "not charging", 4: "full"}
LOCATION_TYPES = {0: "location", 1: "charger", 2: "shelf_home", 3: "slam_marker"}


def pose_dict(p) -> dict:
    return {"x": float(p.x), "y": float(p.y), "theta": float(p.theta)}


def command_dict(command, names: dict | None = None) -> dict | None:
    """{"type": "MOVE_SHELF", "args": {...}} for a pb2.Command; None if empty.

    `names` maps location / shelf ids to names for display.
    """
    names = names or {}
    which = command.WhichOneof("command")
    if not which:
        return None
    body = getattr(command, which)
    args = {}
    for field, value in body.ListFields():
        v = value
        if field.name.endswith("_id") and isinstance(v, str):
            v = names.get(v, v)
        elif hasattr(v, "ListFields"):
            v = {f.name: x for f, x in v.ListFields()}
        args[field.name] = v
    return {"type": which.removesuffix("_command").upper(), "args": args}


def map_dict(m) -> dict:
    return {"name": m.name, "resolution": float(m.resolution), "width": int(m.width),
            "height": int(m.height), "origin": pose_dict(m.origin)}


def layout_dict(locations, shelves) -> dict:
    return {
        "locations": [{"id": loc.id, "name": loc.name, "type": LOCATION_TYPES.get(loc.type, "location"),
                       "pose": pose_dict(loc.pose)} for loc in locations],
        "shelves": [{"id": s.id, "name": s.name, "pose": pose_dict(s.pose),
                     "size": {"width": float(s.size.width), "depth": float(s.size.depth)},
                     "home_location_id": s.home_location_id} for s in shelves],
    }


def id_names(layout: dict) -> dict:
    return {x["id"]: x["name"] for x in layout["locations"] + layout["shelves"]}


def parse_command(body: dict):
    """(client method name, positional args) for a POST /api/command body.

    Raises ValueError for anything the dashboard does not send.
    """
    kind = body.get("type")

    def need(key):
        v = body.get(key)
        if not isinstance(v, str) or not v:
            raise ValueError(f"{kind} needs {key!r}")
        return v

    if kind == "move_to_location":
        return "move_to_location", (need("location_id"),)
    if kind == "move_shelf":
        return "move_shelf", (need("shelf_id"), need("location_id"))
    if kind == "return_shelf":
        return "return_shelf", (body.get("shelf_id") or "",)
    if kind in ("return_home", "undock_shelf"):
        return kind, ()
    if kind == "move_to_pose":
        try:
            x, y, yaw = (float(body[k]) for k in ("x", "y", "yaw"))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError("move_to_pose needs numbers x, y, yaw") from e
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("move_to_pose needs finite x, y, yaw")
        return "move_to_pose", (x, y, yaw)
    if kind == "cancel":
        return "cancel_command", ()
    raise ValueError(f"unknown command type {kind!r}")


class Dashboard:
    """Reads the robot through `client` and caches what rarely changes."""

    def __init__(self, client):
        self.client = client
        self.lock = threading.Lock()
        self._layout = None
        self._map = None  # (map dict, png bytes)
        self._error_titles = None

    def layout(self, refresh: bool = False) -> dict:
        with self.lock:
            if self._layout is None or refresh:
                self.client.update_resolver()  # move_shelf etc. resolve ids through it
                self._layout = layout_dict(self.client.get_locations(), self.client.get_shelves())
            return self._layout

    def map(self, refresh: bool = False):
        with self.lock:
            if self._map is None or refresh:
                m = self.client.get_png_map()
                self._map = (map_dict(m), bytes(m.data))
            return self._map

    def error_titles(self) -> dict:
        if self._error_titles is None:
            try:
                self._error_titles = {code: e.title_en or e.title
                                      for code, e in self.client.get_robot_error_code().items()}
            except Exception:  # noqa: BLE001 - an older robot without the JSON
                self._error_titles = {}
        return self._error_titles

    def error_list(self, codes) -> list:
        titles = self.error_titles()
        return [{"code": int(c), "title": titles.get(int(c), "")} for c in codes]

    def state(self) -> dict:
        c = self.client
        names = id_names(self.layout())
        battery, supply = c.get_battery_info()
        state, running = c.get_command_state()
        result, last = c.get_last_command_result()
        shelf = c.get_moving_shelf_id()
        cmd = command_dict(running, names)
        return {
            "pose": pose_dict(c.get_robot_pose()),
            "battery": {"percentage": float(battery), "status": POWER_SUPPLY.get(int(supply), "unknown")},
            # Idle reads as PENDING with no command, on the real robot too.
            "command": {"state": COMMAND_STATES.get(int(state), "unspecified") if cmd else "idle",
                        "command": cmd},
            "last_result": {"command": command_dict(last, names), "success": bool(result.success),
                            "error": self.error_list([result.error_code]) if result.error_code else []},
            "moving_shelf": names.get(shelf, shelf) or None,
            "errors": self.error_list(c.get_error()),
            "manual_control": bool(c.get_manual_control_enabled()),
        }

    def history(self) -> list:
        names = id_names(self.layout())
        out = []
        for h in self.client.get_history_list():
            out.append({"id": h.id, "command": command_dict(h.command, names), "success": bool(h.success),
                        "error": self.error_list([h.error_code]) if h.error_code else [],
                        "time": int(h.command_executed_time)})
        return sorted(out, key=lambda h: h["time"], reverse=True)

    def command(self, body: dict) -> dict:
        method, args = parse_command(body)
        fn = getattr(self.client, method)
        if method == "cancel_command":
            result, _ = fn()
        else:
            result = fn(*args, wait_for_completion=False)
        return {"success": bool(result.success),
                "error": self.error_list([result.error_code]) if result.error_code else []}


def make_handler(dash: Dashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # keep the terminal quiet; errors still print
            pass

        def _send(self, body: bytes, ctype: str, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, status=HTTPStatus.OK):
            self._send(json.dumps(obj).encode(), "application/json", status)

        def _guard(self, fn):
            try:
                fn()
            except ValueError as e:
                self._json({"error": str(e)}, HTTPStatus.BAD_REQUEST)
            except Exception as e:  # noqa: BLE001 - gRPC down, robot restarting, ...
                self._json({"error": f"{type(e).__name__}: {e}"}, HTTPStatus.BAD_GATEWAY)

        def do_GET(self):
            path, _, query = self.path.partition("?")
            refresh = "refresh=1" in query
            if path == "/":
                self._send(PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._guard(lambda: self._json(dash.state()))
            elif path == "/api/layout":
                self._guard(lambda: self._json(dash.layout(refresh)))
            elif path == "/api/history":
                self._guard(lambda: self._json(dash.history()))
            elif path == "/api/map":
                self._guard(lambda: self._json(dash.map(refresh)[0]))
            elif path == "/api/map.png":
                self._guard(lambda: self._send(dash.map()[1], "image/png"))
            else:
                self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self):
            if self.path != "/api/command":
                self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                return
            n = int(self.headers.get("Content-Length") or 0)

            def run():
                try:
                    body = json.loads(self.rfile.read(n) or b"{}")
                except json.JSONDecodeError as e:
                    raise ValueError(f"bad JSON: {e}") from e
                self._json(dash.command(body if isinstance(body, dict) else {}))

            self._guard(run)

    return Handler


def serve(client, host: str = "127.0.0.1", port: int = 8080) -> ThreadingHTTPServer:
    return ThreadingHTTPServer((host, port), make_handler(Dashboard(client)))
