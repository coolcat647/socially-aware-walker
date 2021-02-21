#!/usr/bin/env python3

import os
import sys
import gdown

COLOR_RED = "\033[0;31m"
COLOR_GREEN = "\033[0;32m"
COLOR_YELLOW = "\033[0;33m"
COLOR_NC = "\033[0m"

weights_dict_list = [
    {"url":"https://drive.google.com/uc?id=1cewMfusmPjYWbrnuJRuKhPMwRe_b9PaT", 
    "name": "yolov4.weights"},
    {"url":"https://drive.google.com/uc?id=1g7YKid4oyZzWroo7QhZPSzPmb_c2jwhW", 
    "name": "yolov4-tiny.weights"},
]

current_dir = os.path.dirname(__file__)

for idx, weights_dict in enumerate(weights_dict_list):
    filepath = os.path.join(current_dir, weights_dict["name"])
    if os.path.isfile(filepath):
        print(COLOR_YELLOW + "file: " + weights_dict["name"] + " is existed." + COLOR_NC)
        continue
    else:
        print(COLOR_GREEN + "Downloading weights file to " + filepath + COLOR_NC)
        gdown.download(weights_dict["url"], filepath, quiet=False)
