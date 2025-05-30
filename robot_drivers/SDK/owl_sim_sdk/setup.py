#!/usr/bin/env python3
from setuptools import setup, find_packages

with open("README.md","r") as f:
    description = f.read()


setup(
    name="owl_robot_sdk",
    version="0.13",
    description="A python client library to operate OWL Robot in Gazebo and MoveIt.",
    author="Lentin Joseph",
    author_email="lentin@orangewood.co",
    license="Orangewood Labs Inc. ",
    packages=find_packages(exclude=["tests", "docs", "examples"]),
    install_requires=[""],

    long_description=description,
    long_description_content_type="text/markdown"
)

