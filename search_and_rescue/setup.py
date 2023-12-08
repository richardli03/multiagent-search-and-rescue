from setuptools import find_packages, setup

package_name = "search_and_rescue"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/start_multi.py"]),
        ("share/" + package_name, ["launch/bringup_multi.py"]),
        ("share/" + package_name, ["launch/launch_map_server.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="richard",
    maintainer_email="rli@olin.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "multimove = search_and_rescue.move_multi:main",
            "load_map = search_and_rescue.load_map:main",
            "occupancy_field = search_and_rescue.occupancy_field:main",
            "agent = search_and_rescue.agent:main",
        ],
    },
)
