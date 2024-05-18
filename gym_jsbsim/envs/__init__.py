from os import listdir
from os import path
import importlib


dir_path = path.dirname(path.realpath(__file__))
ENVS_FILES = [f for f in listdir(dir_path) if path.isfile(path.join(dir_path, f))]
TASKS_NAMES = {}
for f in ENVS_FILES:
    name_file = f.split(".")[0]
    if name_file[-4:] == "task":
        name_class = name_file.title().replace("_", "")
        TASKS_NAMES[name_file] = name_class

TASKS = {}
for f in TASKS_NAMES:
    module = importlib.import_module("gym_jsbsim.envs." + f)
    my_class = getattr(module, TASKS_NAMES[f])
    TASKS[TASKS_NAMES[f]] = my_class
