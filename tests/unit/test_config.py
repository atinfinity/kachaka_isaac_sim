from kachaka_isaac_sim.config import DEFAULT_CONFIG, load_config


def test_default_config_loads():
    cfg = load_config()
    assert cfg.drive.wheel_radius == 0.045
    assert cfg.drive.wheel_separation == 0.2
    assert cfg.lidar.max_range > cfg.lidar.min_range > 0
    assert cfg.imu.rate > 0


def test_explicit_path(tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text("drive:\n  wheel_radius: 0.1\nlist:\n  - {a: 1}\n")
    cfg = load_config(path)
    assert cfg.drive.wheel_radius == 0.1
    assert DEFAULT_CONFIG.exists()
