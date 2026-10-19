import json
import threading
import urllib.error
import urllib.request
from types import SimpleNamespace as NS

import pytest

from kachaka_isaac_sim.dashboard import Dashboard, command_dict, parse_command, serve


class FakeMsg:
    """Just enough of a protobuf message: ListFields and WhichOneof."""

    def __init__(self, oneof=None, **fields):
        self._oneof = oneof
        self._fields = fields
        for k, v in fields.items():
            setattr(self, k, v)

    def ListFields(self):
        return [(NS(name=k), v) for k, v in self._fields.items()]

    def WhichOneof(self, _):
        return self._oneof


def pose(x=0.0, y=0.0, theta=0.0):
    return NS(x=x, y=y, theta=theta)


EMPTY = FakeMsg()


class FakeClient:
    def __init__(self):
        self.calls = []
        self.running = FakeMsg("move_shelf_command", move_shelf_command=FakeMsg(
            target_shelf_id="S01", destination_location_id="L01"))

    def update_resolver(self):
        self.calls.append(("update_resolver",))

    def get_locations(self):
        return [NS(id="L01", name="kitchen", type=0, pose=pose(1, 2, 0)),
                NS(id="home", name="charger", type=1, pose=pose(0, 0, 3.14))]

    def get_shelves(self):
        return [NS(id="S01", name="shelf", pose=pose(2, 0, 0), size=NS(width=0.4, depth=0.35),
                   home_location_id="L02")]

    def get_robot_pose(self):
        return pose(0.5, -0.25, 1.0)

    def get_battery_info(self):
        return 87.5, 2

    def get_command_state(self):
        return 2, self.running

    def get_last_command_result(self):
        return NS(success=False, error_code=10253), FakeMsg("return_home_command", return_home_command=EMPTY)

    def get_moving_shelf_id(self):
        return "S01"

    def get_error(self):
        return [14605]

    def get_manual_control_enabled(self):
        return False

    def get_robot_error_code(self):
        return {10253: NS(title="t", title_en="Navigation failed"), 14605: NS(title="ja", title_en="")}

    def get_history_list(self):
        return [NS(id="a", command=FakeMsg("return_home_command", return_home_command=EMPTY), success=True,
                   error_code=0, command_executed_time=100),
                NS(id="b", command=self.running, success=False, error_code=10253, command_executed_time=200)]

    def get_png_map(self):
        return NS(name="room", resolution=0.05, width=4, height=3, origin=pose(-1, -2), data=b"\x89PNG")

    def __getattr__(self, name):  # commands
        def call(*args, **kw):
            self.calls.append((name, args, kw))
            ok = NS(success=True, error_code=0)
            return (ok, EMPTY) if name == "cancel_command" else ok
        return call


def test_command_dict_names_ids():
    c = FakeClient()
    assert command_dict(c.running, {"S01": "shelf", "L01": "kitchen"}) == {
        "type": "MOVE_SHELF", "args": {"target_shelf_id": "shelf", "destination_location_id": "kitchen"}}
    assert command_dict(EMPTY) is None


def test_parse_command():
    assert parse_command({"type": "move_shelf", "shelf_id": "S01", "location_id": "L01"}) == (
        "move_shelf", ("S01", "L01"))
    assert parse_command({"type": "move_to_pose", "x": "1", "y": 2, "yaw": 0}) == ("move_to_pose", (1.0, 2.0, 0.0))
    assert parse_command({"type": "return_shelf"}) == ("return_shelf", ("",))
    assert parse_command({"type": "cancel"}) == ("cancel_command", ())
    for bad in ({"type": "move_shelf", "shelf_id": "S01"}, {"type": "move_to_pose", "x": 1},
                {"type": "move_to_pose", "x": "nan", "y": 0, "yaw": 0}, {"type": "speak"}, {}):
        with pytest.raises(ValueError):
            parse_command(bad)


def test_state_and_history():
    d = Dashboard(FakeClient())
    s = d.state()
    assert s["pose"] == {"x": 0.5, "y": -0.25, "theta": 1.0}
    assert s["battery"] == {"percentage": 87.5, "status": "discharging"}
    assert s["command"]["state"] == "running"
    assert s["command"]["command"]["args"]["target_shelf_id"] == "shelf"
    assert s["last_result"]["error"] == [{"code": 10253, "title": "Navigation failed"}]
    assert s["moving_shelf"] == "shelf"
    assert s["errors"] == [{"code": 14605, "title": "ja"}]
    h = d.history()
    assert [x["id"] for x in h] == ["b", "a"]  # newest first
    assert h[0]["error"][0]["code"] == 10253


def test_commands_do_not_wait():
    c = FakeClient()
    d = Dashboard(c)
    assert d.command({"type": "move_to_location", "location_id": "L01"})["success"]
    assert c.calls[-1] == ("move_to_location", ("L01",), {"wait_for_completion": False})
    assert d.command({"type": "cancel"})["success"]
    assert c.calls[-1] == ("cancel_command", (), {})


@pytest.fixture
def server():
    srv = serve(FakeClient(), "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def fetch(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    try:
        with urllib.request.urlopen(urllib.request.Request(url, data=data), timeout=5) as r:
            return r.status, r.headers["Content-Type"], r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers["Content-Type"], e.read()


def test_http(server):
    status, ctype, page = fetch(server + "/")
    assert status == 200 and ctype.startswith("text/html") and b"<canvas" in page
    status, _, body = fetch(server + "/api/state")
    assert status == 200 and json.loads(body)["battery"]["percentage"] == 87.5
    status, _, body = fetch(server + "/api/layout")
    assert [loc["type"] for loc in json.loads(body)["locations"]] == ["location", "charger"]
    assert json.loads(fetch(server + "/api/map")[2])["origin"] == {"x": -1.0, "y": -2.0, "theta": 0.0}
    assert fetch(server + "/api/map.png")[1] == "image/png"
    assert json.loads(fetch(server + "/api/command", {"type": "return_home"})[2])["success"]
    assert fetch(server + "/api/command", {"type": "nope"})[0] == 400
    assert fetch(server + "/api/nope")[0] == 404
