UNAME           = $(shell uname)

ifeq ($(UNAME),Linux)
PYTHON = python
endif

ifeq ($(UNAME),Darwin)
PYTHON = python3
R = /usr/local/bin/R
endif

# use Butterworth?
SMOOTH = False

# Tobii Glasses 2 sample at 100 Hz
WIDTH = 1920
HEIGHT = 1080
HERTZ = 50
#DIST = 25.59 # 65 cm
#DIST = 23.62 # 60 cm
#DIST = 22.44 # 57 cm
DIST = 22.24 # 56.5 cm
#DIST = 19.68 # 50 cm
#DIST = 15.74 # 40 cm
SCREEN = 22

XTILES = 2
YTILES = 1

SDDIR = ../../card/aruco_lamp/
# no ending / on PRJDIR
PRJDIR = ../../card/aruco_lamp/projects
INDIR = ../../data/aruco_lamp/

VIDDIR = ../../data/aruco_lamp/
PLTDIR = ./plots/
OUTDIR = ./data/
RAWDIR = ./data/raw/

#all: dirs json csv raw demo
all:
	@echo "Pick one: "
	@echo "  make dirs"
	@echo "  make json"
	@echo "  make csv"
	@echo "  make raw"

dirs:
	mkdir -p data
	mkdir -p data/raw
	mkdir -p plots
	mkdir -p figs
	mkdir -p figs/TMS

json:
	$(PYTHON) ./card2json.py --indir=$(PRJDIR) --outdir="./" --outid="../../data/aruco_lamp/"

csv:
	$(PYTHON) ../tobii/tobii_grab_data.py generated.json --convert 0 --file=$(SDDIR)
#	$(PYTHON) ../tobii/tobii_grab_data.py aruco_lamp.json --convert 2 --file=$(SDDIR)

raw:
	$(PYTHON) ./csv2raw.py --indir=$(INDIR) --outdir=$(RAWDIR)

demo:
	$(PYTHON) ./glcam-tobii-aruco.py --vfile=../../data/aruco_lamp/aruco_lamp.mp4 --file=./data/raw/aruco_lamp.raw

process:
	$(PYTHON) ./filter-pl.py --indir=$(RAWDIR) --outdir=$(OUTDIR) \
                                 --width=$(WIDTH) --height=$(HEIGHT) \
                                 --dist=$(DIST) --screen=$(SCREEN) \
                                 --herz=$(HERTZ)

graph:
	$(PYTHON) ./graph.py --smooth=$(SMOOTH) --indir=$(RAWDIR) --imgdir=$(IMGDIR) --dist=$(DIST) --screen=$(SCREEN) --width=$(WIDTH) --height=$(HEIGHT) --hertz=$(HERTZ) --xtiles=$(XTILES) --ytiles=$(YTILES) --outdir=$(OUTDIR) --pltdir=$(PLTDIR)
#	$(PYTHON) ./graph.py --smooth=$(SMOOTH) --indir=$(RAWDIR) --imgdir=$(IMGDIR) --dist=$(DIST) --screen=$(SCREEN) --width=$(WIDTH) --height=$(HEIGHT) --hertz=$(HERTZ) --xtiles=$(XTILES) --ytiles=$(YTILES) --outdir=$(OUTDIR) --pltdir=$(PLTDIR) --image="white-1600x900.png"

collate:
	$(PYTHON) ./collate-vfpt.py

stats:
#	$(R) --vanilla --max-mem-size < hits.R > hits.out
#	$(R) --vanilla --args 3 < tm.R > tm.out

clean: 
	rm -rf *.pyc 
	rm -rf data/
	rm -rf plots/
	rm -rf figs/
	rm -rf *.pdf
	rm -rf *.out
	rm -rf *.json
	rm -rf __pycache__
