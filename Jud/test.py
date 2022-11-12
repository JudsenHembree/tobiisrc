#!/usr/bin/env python3

# system includes
import json
import sys
import getopt
import os
import numpy as np
from subprocess import Popen, PIPE

# local includes

def main():
    f = open("test.txt", "w")
    f.write("wombo")
    f.close()

if __name__ == "__main__":
    main()
