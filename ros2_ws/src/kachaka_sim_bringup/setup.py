from glob import glob

from setuptools import setup

package_name = "kachaka_sim_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/behavior_trees", glob("behavior_trees/*.xml")),
        (f"share/{package_name}/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    license="Apache-2.0",
    entry_points={"console_scripts": [
        f"nav_bridge = {package_name}.nav_bridge:main",
        f"footprint_map_filter = {package_name}.footprint_map_filter:main",
        f"command_server = {package_name}.command_server:main",
        f"shelf_markers = {package_name}.shelf_markers:main",
        f"fake_robot = {package_name}.fake_robot:main",
    ]},
)
