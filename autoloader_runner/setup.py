from glob import glob
from setuptools import find_packages, setup

package_name = "autoloader_runner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*launch.[pxy][yma]*")),
        (
            "lib/" + package_name + "/autoloader_data",
            glob("../autoloader_data/__init__.py"),
        ),
        (
            "lib/" + package_name + "/autoloader_data/src",
            glob("../autoloader_data/autoloader_data/src/*.py"),
        ),
        (
            "lib/" + package_name + "/autoloader_data/src",
            glob("../autoloader_data/autoloader_data/src/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="willnatsan",
    maintainer_email="sannatwill@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "runner = autoloader_runner.runner_node:main",
        ],
    },
)
