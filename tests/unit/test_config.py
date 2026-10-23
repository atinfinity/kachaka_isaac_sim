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


def test_base_merges(tmp_path):
    (tmp_path / "base.yaml").write_text("drive:\n  a: 1\n  b: 2\nlist: [1, 2]\n")
    (tmp_path / "over.yaml").write_text("base: base.yaml\ndrive:\n  b: 3\nlist: [9]\n")
    cfg = load_config(tmp_path / "over.yaml")
    assert (cfg.drive.a, cfg.drive.b, cfg.list) == (1, 3, [9])
    assert not hasattr(cfg, "base")


def test_base_loop(tmp_path):
    import pytest

    (tmp_path / "a.yaml").write_text("base: b.yaml\n")
    (tmp_path / "b.yaml").write_text("base: a.yaml\n")
    with pytest.raises(ValueError, match="loop"):
        load_config(tmp_path / "a.yaml")


def test_degraded_example():
    cfg = load_config(DEFAULT_CONFIG.parent / "degraded.yaml")
    assert cfg.drive.wheel_radius == 0.045  # from robot.yaml
    assert cfg.degradation.lidar.dropout > 0
    assert cfg.degradation.lidar.dark_range == 1.5  # kept from robot.yaml
