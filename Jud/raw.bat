set PYTHON=python

set SDDIR=../../card/aruco_lamp/
# no ending / on PRJDIR
set PRJDIR=../../card/aruco_lamp/projects
set INDIR=../../data/aruco_lamp/

set VIDDIR=../../data/aruco_lamp/
set PLTDIR=./plots/
set OUTDIR=./data/
set RAWDIR=./data/raw/

%PYTHON% ./csv2raw.py --indir=%INDIR% --outdir=%RAWDIR%
