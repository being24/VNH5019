import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="VNH5019_driver",
    version="0.0.3",
    author="being24",
    author_email="being24@gmail.com",
    description="VNH5019 driver with pigpio and pca9685 on Raspberry pi",
    install_requires=[
        "PCA9685_wrapper",
    ],
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/being24/VNH5019_driver",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: MIT License",
    ],
)