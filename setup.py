from setuptools import setup, find_packages
from distutils.util import get_platform
from distutils.version import StrictVersion as parse_version
from zipfile import ZipFile
import urllib.request
import shutil
import subprocess
import os


def _get_version_hash():
    """Talk to git and find out the tag/hash of our latest commit"""
    try:
        ver = subprocess.check_output(
            ["git", "describe", "--tags", "--always"], encoding="utf-8"
        )
    except OSError:
        print("Couldn't run git to get a version number for setup.py")
        return
    return ver.strip()


version = _get_version_hash()

if version[:1] == "v":
    version = version[1:]

try:
    from jsbsim import __version__ as jsbsim_version # noqa: F401
except ImportError:
    pass

print("Building wheels {}-{}".format(version, jsbsim_version))

cwd = os.path.dirname(os.path.abspath(__file__))


def write_version_file():
    with open(os.path.join(cwd, "gym_jsbsim", "version.py"), "w") as file:
        file.write("__version__ = '{}'\n".format(version))
        file.write("__jsbsim_version__ = '{}'\n".format(jsbsim_version))


# download a/c data
to_path = "gym_jsbsim/"

urllib.request.urlretrieve(
    "https://github.com/JSBSim-Team/jsbsim/archive/v{}.zip".format(jsbsim_version),
    "v{}.zip".format(jsbsim_version),
)

sub_dirs = ["aircraft", "engine", "systems"]
archive = ZipFile("v{}.zip".format(jsbsim_version))
for file in archive.namelist():
    l = file.split('/')
    if len(l) > 2 and l[1] in sub_dirs:
        print(file)
        archive.extract(file, to_path)

# move aircraft from docs to jsbsim directory
from_path = os.path.join("gym_jsbsim", "docs", "aircraft")
to_path = os.path.join("gym_jsbsim", "jsbsim-{}".format(jsbsim_version), "aircraft")
for aircraft in os.listdir(from_path):
    for f in os.listdir(os.path.join(from_path, aircraft)):
        shutil.copyfile(
            os.path.join(from_path, aircraft, f), os.path.join(to_path, aircraft, f)
        )

with open("README.md") as f:
    long_description = f.read()

requirements = [
    "jsbsim>=" + jsbsim_version,
    "folium>=0.10.1",
    "geographiclib>=1.50",
    "gym>=0.15.7",
    "shapely>=1.7.1",
]

if __name__ == "__main__":
    write_version_file()
    setup(
        name="gym_jsbsim",
        version=version,
        author="Guillaume Alleon",
        author_email="guillaume.alleon@gmail.com",
        url="https://github.com/galleon/gym-jsbsim",
        description="Gym JSBSim environment",
        long_description=long_description,
        long_description_content_type="text/markdown",
        license="LGPL 2.1",
        python_requires=">3.6",
        setup_requires=["pytest-runner"],
        tests_require=["pytest"],
        packages=find_packages(exclude=("docs", "tests", "notebooks")),
        package_data={
            "gym_jsbsim": [
                "jsbsim-{}/aircraft/*/*.xml".format(jsbsim_version),
                "jsbsim-{}/systems/*.xml".format(jsbsim_version),
                "jsbsim-{}/engine/*.xml".format(jsbsim_version),
            ]
        },
        install_requires=requirements,
        classifiers=[
            "Intended Audience :: Developers",
            "Intended Audience :: Education",
            "Intended Audience :: Science/Research",
            "Operating System :: POSIX :: Linux",
            "Programming Language :: Python :: 3 :: Only",
            "License :: OSI Approved :: MIT License",
            "Topic :: Scientific/Engineering :: Artificial Intelligence",
            "Topic :: Games/Entertainment :: Simulation",
            "Topic :: Software Development :: Libraries",
        ],
    )
