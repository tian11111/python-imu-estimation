from setuptools import find_packages, setup


setup(
    name="imu-estimation",
    version="0.1.0",
    description="MPU6050/MPU6500 six-axis MEKF for Raspberry Pi cars",
    package_dir={"": "src"},
    packages=find_packages("src"),
    python_requires=">=3.7",
    install_requires=["numpy>=1.17"],
)
