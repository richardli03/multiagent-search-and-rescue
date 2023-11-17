from setuptools import find_packages, setup

package_name = "search-and-rescue"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/launch.py"]),
        ("share/" + package_name, ["launch/bringup_multi.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="richard",
    maintainer_email="rli@olin.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
