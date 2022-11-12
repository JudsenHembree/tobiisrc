#!/usr/bin/env python

import sys, os, math

# these are encoded as "'Recording': ['risk', 'contrast', 'survey']" where
# risk is low, high, contrast is low, high, survey is a 4-digit code
condict = { \
'Recording012':	['low',	'high', 6027],\
'Recording013':	['high','high', 8533],\
'Recording014':	['low',	'high', 8627],\
'Recording015':	['high','low', 6173],\
'Recording016':	['high','high', 3202],\
'Recording017':	['low',	'low', 9452],\
'Recording018':	['low',	'low', 1604],\
'Recording019':	['low',	'high', 5855],\
'Recording020':	['high','low', 9030],\
'Recording021':	['low',	'high', 6319],\
'Recording022':	['high','low', 7347],\
'Recording023':	['high','high', 9312],\
'Recording024':	['low',	'low', 3503],\
'Recording025':	['high','low', 9105],\
'Recording026':	['high','low', 8130],\
'Recording027':	['low',	'high', 3763],\
'Recording028':	['high','high', 3455],\
'Recording029':	['low',	'low', 5278],\
'Recording030':	['high','low', 1919],\
'Recording031':	['high','high', 8627],\
'Recording032':	['low',	'low', 2237],\
'Recording033':	['low',	'high', 9113],\
'Recording034':	['high','low', 4495],\
'Recording035':	['low',	'high', 9429],\
'Recording036':	['high','high', 2015],\
'Recording037':	['high','low', 2327],\
'Recording038':	['low',	'low', 1391],\
'Recording039':	['high','high', 5005],\
'Recording040':	['high','low', 8718],\
'Recording041':	['low',	'high', 2537],\
'Recording042':	['high','low', 6517],\
'Recording043':	['low',	'high', 9127],\
'Recording044':	['low',	'high', 1891],\
'Recording045':	['high','low', 1362],\
'Recording046':	['high','high', 5458],\
'Recording047':	['low',	'low', 5431],\
'Recording048':	['low',	'low', 3299],\
'Recording049':	['high','high', 1334],\
'Recording050':	['low',	'low', 1402],\
'Recording051':	['high','high', 9519],\
'Recording052':	['low',	'low', 8537],\
'Recording053':	['high','high', 4397],\
'Recording054':	['high','low', 1845],\
'Recording055':	['low',	'high', 5352],\
'Recording056':	['low',	'low', 2552],\
'Recording057':	['high','high', 2422],\
'Recording058':	['low',	'high', 6050],\
'Recording059':	['low',	'low', 4374],\
'Recording060':	['high','high', 2650],\
'Recording061':	['low',	'low', 9115],\
'Recording062':	['high','high', 8823],\
'Recording063':	['high','low', 4736],\
'Recording064':	['low',	'high', 9040],\
'Recording065':	['high','low', 4534],\
'Recording066':	['low',	'low', 5249],\
'Recording067':	['low',	'high', 8233],\
'Recording072':	['high','high', 5134],\
'Recording073':	['high','low', 9309],\
'Recording074':	['low',	'high', 1955],\
'Recording075':	['low',	'low', 7180],\
'Recording076':	['high','low', 3485],\
'Recording077':	['high','high', 4321],\
'Recording078':	['low',	'high', 1644],\
'Recording079':	['high','low', 3999],\
'Recording080':	['low',	'low', 6911],\
'Recording081':	['low',	'high', 5936],\
'Recording082':	['high','high', 8939],\
'Recording083':	['high','low', 3665],\
'Recording084':	['low',	'low', 5066],\
'Recording085':	['high','high', 9676],\
'Recording086':	['low',	'low', 5363],\
'Recording087':	['low',	'high', 2563],\
'Recording088':	['high','low', 2025],\
'Recording089':	['high','high', 5668],\
'Recording090':	['low',	'low', 7646],\
'Recording091':	['low',	'high', 7243],\
'Recording092':	['low',	'low', 1120],\
'Recording093':	['high','high', 7054],\
'Recording094':	['low',	'low', 9053],\
'Recording095':	['low',	'high', 7832],\
'Recording096':	['low',	'high', 6007],\
'Recording097':	['high','low', 4231],\
'Recording098':	['high','low', 6998],\
'Recording099':	['low',	'high', 8907],\
'Recording100':	['low',	'low', 3231],\
'Recording101':	['high','high', 1782],\
'Recording102':	['high','low', 5822],\
'Recording103':	['low',	'high', 9253],\
'Recording104':	['low',	'low', 6495],\
'Recording105':	['high','high', 7635],\
'Recording106':	['low',	'low', 9221],\
'Recording107':	['high','high', 2265],\
'Recording108':	['high','low', 7373],\
'Recording109':	['high','high', 2905],\
'Recording110':	['low',	'high', 3253],\
'Recording111':	['high','low', 1326],\
'Recording112':	['low',	'low', 5406],\
'Recording113':	['high','high', 3193],\
'Recording114':	['low',	'low', 2759],\
'Recording115':	['high','high', 8965],\
'Recording116':	['low',	'low', 8665],\
'Recording117':	['high','high', 2414],\
'Recording118':	['low',	'high', 6363],\
'Recording119':	['low',	'low', 4169],\
'Recording120':	['high','low', 9468],\
'Recording121':	['high','high', 8888],\
'Recording122':	['low',	'high', 3412],\
'Recording123':	['high','low', 3863],\
'Recording124':	['low',	'low', 3941],\
'Recording125':	['low',	'high', 9802],\
'Recording126':	['high','high', 4648],\
'Recording127':	['high','low', 3328],\
'Recording128':	['low',	'high', 4190],\
'Recording129':	['high','low', 7324],\
'Recording130':	['low',	'high', 3133],\
'Recording131':	['low',	'low', 5866],\
'Recording132':	['high','low', 5655],\
'Recording133':	['low',	'high', 6268],\
'Recording134':	['high','high', 3950],\
'Recording135':	['high','low', 8600],\
'Recording136':	['low',	'high', 1170],\
'Recording137':	['high','low', 4314],\
'Recording138':	['low',	'high', 6197],\
'Recording139':	['high','low', 7834],\
'Recording140':	['low',	'low', 4821],\
'Recording141':	['low',	'high', 6687],\
'Recording142':	['high','low', 9370],\
}

def catCSVFile(infile,df,ct):
  try:
    f = open(infile,'r')
  except IOError:
    print("Can't open file: " + infile)
    return

  base = os.path.basename(infile)

  print("Processing: ", infile, "[", base, "]")

  # TUTORIAL NOTE: edit this to properly parse the .dat files
  # extract subj id
  # expected: "RecordingXXX", just want "XXX"
# subj = base.split('-')[0][9:]
  # use full "RecordingXXX" name so we can use this as key into condict
  subj = base.split('-')[0]

  # some files are RecordingXXX_1, remove the _1 crap
  if '_' in subj:
    subj = subj.split('_')[0]

  # look up condict to see if this subj is our list
  if subj in condict:
    risk = condict[subj][0]
    contrast = condict[subj][1]

    print("subj, risk, contrast: ", \
           subj, risk, contrast)
  else:
    # don't process whatever is not in condict
    print("%s not in condict" % (subj))
    return ct

  # read lines, throwing away first one (header)
# linelist = f.readlines()
# linelist = f.read().split('\r')
  linelist = f.read().splitlines()
  header = linelist[0].split(',')
  linelist = linelist[1:]

  # frame,timestamp,cond,face_i,[{feature_1,feature_2.,,,}],dur
  FRAME = 0
  TIMESTAMP = 1
  AOI = 2
  DUR = 3

  # reset vars
  frame = []
  timestamp = []
  aoi = []
  duration  = ''

  for line in linelist:
    entry = line.split(',')

    # start of new fixation
    if entry[DUR] != duration:

#     print "frame: ", frame
      if len(frame) == 0:
        mc_frame = ' '
      elif len(frame) == 1:
        mc_frame = "%s,%s" % (frame[0],frame[0])
      else:
        mc_frame = "%s,%s" % (frame[0],frame[-1])
#     print("mc_frame: ", mc_frame)

#     print("timestamp: ", timestamp)
      if len(timestamp) == 0:
        mc_timestamp = ' '
      elif len(timestamp) == 1:
        mc_timestamp = "%s,%s" % (timestamp[0],timestamp[0])
      else:
        mc_timestamp = "%s,%s" % (timestamp[0],timestamp[-1])
#     print("mc_timestamp: ", mc_timestamp)

#     print("aoi: ", aoi)
      if len(aoi) == 0:
        mc_aoi = ' '
      elif len(aoi) == 1:
        mc_aoi = "%s" % (aoi[0])
      else:
        mc_aoi = "%s" % max(aoi, key=aoi.count).strip()
#     print("mc_aoi: ", mc_aoi)

      # if frame is not empty
      if frame:
        # edit this to properly output the .csv files
        str = "%s,%s,%s,%s,%s,%s,%s" % ( \
                                        subj[9:], \
                                        risk,\
                                        contrast,\
                                        mc_frame,\
                                        mc_timestamp,\
                                        mc_aoi,\
                                        duration)
        print(str, file=df)
        ct += 1

      # reset vars
      frame = []
      timestamp = []
      aoi = []
      duration  = ''

    # frame,timestamp,cond,face_i,[{feature_1,feature_2.,,,}],dur
    # get line elements
    frame.append(entry[FRAME])
    timestamp.append(entry[TIMESTAMP])
    aoi.append(entry[AOI])
    duration  = entry[DUR]

  return ct

###############################################################################

# TUTORIAL NOTE: edit this to properly output the .csv file header
# clear out output file
df = open("vfpt.csv",'w')
print("recording,risk,contrast,sf,ef,ts,te,aoi,dur", file=df)

dir = './data/'

# find all files in dir with .csv extension
lst = [a for a in os.listdir(dir) if a.endswith('-vfpt.dat')]

lineno = 1

for item in lst:

  file = dir + item
  print('Processing ', file)

  print("lineno: ", lineno)

  # cat csv files into one
  lineno = catCSVFile(file,df,lineno)

df.close()
