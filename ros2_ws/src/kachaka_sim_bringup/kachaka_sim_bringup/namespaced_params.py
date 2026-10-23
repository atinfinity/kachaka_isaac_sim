"""nav2_params.yaml for a robot in a ROS namespace (robot_nav.launch.py).

nav2_params.yaml is written for the single robot: Nav2 at the root, the
Kachaka topics under /kachaka and TF frames without a prefix. Robot i of a
fleet (run_sim.py --robot ..., repeated) has Nav2 under /kachaka_i, its topics
under /kachaka_i and frames kachaka_i/odom, kachaka_i/base_footprint. Nav2's
RewrittenYaml rewrites every key of a given name, but the same key holds
different values here (global_frame is odom in the local costmap and map in
the global one), so this rewrites values instead:

- base_footprint, and odom under a frame key: prefixed;
- /kachaka/...: under the robot's Kachaka namespace;
- other absolute topics: under the robot's namespace too (/nav_map ->
  /kachaka_i/nav_map), except world-wide ones (SHARED_TOPICS). Not relative:
  a costmap resolves those in its own namespace (/kachaka_i/global_costmap);
- map and everything else: unchanged (all robots share one map frame).

AMCL's set_initial_pose is turned off: the map frame's origin is robot 0's
spawn, not this robot's, so its start pose comes on its initialpose topic.
"""

import tempfile
from pathlib import Path

import yaml

SOURCE_NAMESPACE = "kachaka"
SHARED_TOPICS = {"/parked_shelves"}  # published once for the whole world (command_server of robot 0)


def _frame_key(key: str) -> bool:
    return key.endswith(("frame", "frame_id"))


def _topic_key(key: str) -> bool:
    return key == "topic" or key.endswith("_topic")


def rewrite_value(key: str, value, kachaka_namespace: str, frame_prefix: str):
    if not isinstance(value, str):
        return value
    if value == "base_footprint" or (value == "odom" and _frame_key(key)):
        return frame_prefix + value
    source = f"/{SOURCE_NAMESPACE}/"
    if value.startswith(source):
        return f"/{kachaka_namespace}/{value[len(source):]}"
    if value.startswith("/") and _topic_key(key) and value not in SHARED_TOPICS:
        return f"/{kachaka_namespace}{value}"
    return value


def rewrite(params, kachaka_namespace: str, frame_prefix: str, key: str = ""):
    """`params` (parsed YAML) with the values rewritten for the robot."""
    if isinstance(params, dict):
        return {k: rewrite(v, kachaka_namespace, frame_prefix, str(k)) for k, v in params.items()}
    if isinstance(params, list):
        return [rewrite(v, kachaka_namespace, frame_prefix, key) for v in params]
    return rewrite_value(key, params, kachaka_namespace, frame_prefix)


def namespaced_params_file(src: str, namespace: str, out_dir: str | None = None) -> str:
    """Write `src` rewritten for the robot in `namespace` (its Kachaka
    namespace too, frame prefix "<namespace>/") and return the new path.
    An empty namespace returns `src`."""
    ns = namespace.strip("/")
    if not ns:
        return src
    params = rewrite(yaml.safe_load(Path(src).read_text()), ns, f"{ns}/")
    amcl = (params.get("amcl") or {}).get("ros__parameters")
    if amcl is not None:
        amcl["set_initial_pose"] = False
    with tempfile.NamedTemporaryFile("w", prefix=f"nav2_params_{ns}_", suffix=".yaml", dir=out_dir,
                                     delete=False) as out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name
