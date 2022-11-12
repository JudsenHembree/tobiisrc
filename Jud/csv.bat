set PYTHON=python

set SDDIR=../../card/aruco_lamp/
# no ending / on PRJDIR
set PRJDIR=../../card/aruco_lamp/projects
set INDIR=../../data/aruco_lamp/

%PYTHON% ../tobii/tobii_grab_data.py aruco_lamp.json --convert 0 --file=%SDDIR%
