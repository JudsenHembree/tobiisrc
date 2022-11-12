#!/usr/bin/env python2

# system includes
import sys
import os
import numpy as np

# local includes
from aoi import AOI

def main(argv):

  indir = "../../exp/jaconde/"

  aoi = AOI([179,764,179+211,764+108])

  aoi.dump()

  print("in" if aoi.inside(180.0,780) else "out")
  print("in" if aoi.inside(0.0,0.0) else "out")

if __name__ == "__main__":
  main(sys.argv[1:])
