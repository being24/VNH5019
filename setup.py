import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="TA7291P_driver",
    version="0.0.2",
    author="being24",
    author_email="being24@gmail.com",
    description="TA7291P driver with pigpio and pca9685 on Raspberry pi",
    install_requires=[
        "PCA9685_wrapper",
    ],
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/being24/TA7291P_driver",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: MIT License",
    ],
)