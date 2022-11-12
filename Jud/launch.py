#!/usr/bin/env python3

# system includes
import json
import time
import shlex
import sys
import getopt
import os
import glob
import numpy as np
from subprocess import Popen, PIPE

# local includes





def launchprocess():
    print("not yet supported")

def launchgraph():
    print("not yet supported")

def launchcollate():
    print("not yet supported")

def launchclean():
    print("not yet supported")

def launchstats():
    print("not yet supported")

def processCfg(file):
    print("processing config")
    cfg = open(file)
    data = json.load(cfg)
    pretty = json.dumps(data, indent=4)
    print(pretty)
    return data

def dirs():
    dirertories = ["data", "data/raw", "plots", "figs", "figs/TMS"]
    for d in dirertories:
        try:
            os.mkdir(d)
        except:
            files = glob.glob(d + '/*')
            for file in files:
                try:
                    os.remove(file)
                except IsADirectoryError:
                    print("Skipping directory " + file)

            

def test():
    print("test")

def checkKey(opts):
    for opt, _ in opts:
        if opt == '--cfg':
            return True
    return False

def pickOne():
    print("Pick one:")
    print("\t./launch.py --dirs")
    print("\t./launch.py --json")
    print("\t./launch.py --csv")
    print("\t./launch.py --raw")

def launchjson(py, CFG):
            print("json")
            arg = py + " ./card2json.py --indir='" + CFG["locations"]["prj"] + "' --outdir='./' --outid=" +  CFG["locations"]["vid"]
            print(arg)
            with Popen(shlex.split(arg), shell=False, stdout=PIPE) as process:
                while True:
                    output = process.stdout.readline()
                    if process.poll() is not None:
                        break
                    if output:
                        print(output.decode('utf-8'))

            time.sleep(1)
            

def launchcsv(py, CFG):
            print("csv")
            arg = py + " ../tobii/tobii_grab_data.py generated.json --convert 0 --file=" + CFG["locations"]["sd"] 
            print(arg)
            with Popen(shlex.split(arg), shell=False, stdout=PIPE) as process:
                while True:
                    output = process.stdout.readline()
                    if process.poll() is not None:
                        break
                    if output:
                        print(output.decode('utf-8'))
            time.sleep(1)

def launchraw(py, CFG):
            print("raw")
            arg = py + " ./csv2raw.py --indir=" + CFG["locations"]["in"] + " --outdir=" + CFG["locations"]["out"]
            print(arg)
            with Popen(shlex.split(arg), shell=False, stdout=PIPE) as process:
                while True:
                    output = process.stdout.readline()
                    if process.poll() is not None:
                        break
                    if output:
                        print(output.decode('utf-8'))
            time.sleep(1)

def launchdemo(py, CFG):
            print("demo")
            arg = py + " ./glcam-tobii-aruco.py --vfile=" + CFG["locations"]["video"] + " --file=" + CFG["locations"]["rawfile"]
            print(arg)
            with Popen(shlex.split(arg), shell=False, stdout=PIPE) as process:
                while True:
                    output = process.stdout.readline()
                    if process.poll() is not None:
                        break
                    if output:
                        print(output.decode('utf-8'))
            time.sleep(1)

def generatecfg(location):
    print("generating cfg")
    print(location)

def main(argv):
    CFG = {}
    py = "python3"
    try:
        opts, args = getopt.getopt(argv, '', \
                 ['dirs','json','csv','raw',\
                  'demo','process','graph',\
                  'collate','stats','clean',\
                  'all','cfg=','base='])
        print(opts)
    except getopt.GetoptError as err:
        print(err)
        pickOne()
        exit()

    for opt, arg in opts:
        if(opt == '--base'):
            generatecfg(arg)

    if not checkKey(opts):
        print("must provide cfg")
        exit()

    for opt, arg in opts:
        if(opt == '--cfg'):
            CFG = processCfg(arg) 

    for opt, arg in opts:
        if(opt == '--all'):
            dirs()
            launchjson(py, CFG)
            launchcsv(py, CFG)
            launchraw(py, CFG)
            launchdemo(py, CFG)
        if(opt == '--dirs'):
            dirs()
        if opt == '--json':
            launchjson(py, CFG)
            exit()
        if opt == '--csv':
            launchcsv(py, CFG)
            exit()
        if opt == '--raw':
            launchraw(py, CFG)
            exit()
        if opt == '--demo':
            launchdemo(py, CFG)
            exit()
        if(opt == '--process'):
            launchprocess()
        if(opt == '--graph'):
            launchgraph()
        if(opt == '--collate'):
            launchcollate()
        if(opt == '--stats'):
            launchstats()
        if(opt == '--clean'):
            launchclean()
        else:
            exit()


if __name__ == "__main__":
  main(sys.argv[1:])
