from setuptools import setup

package_name = "racelib"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="racecar team",
    maintainer_email="team@example.com",
    description="穿越火线算法核心库",
    license="MIT",
)
