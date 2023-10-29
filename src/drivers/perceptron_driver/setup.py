from setuptools import find_packages, setup

package_name = "perceptron_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pedro Soares",
    maintainer_email="pmbs.123@gmail.com",
    description="A driver to communicate with arduino to move the robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver_node = perceptron_driver.driver_node:main"
        ],
    },
)
