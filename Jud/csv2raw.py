#!/usr/bin/env python

import sys,os,getopt,glob
import locale
import numpy as np

def usage():
  print("Usage: python csv2raw.py " \
        " --indir=? -outdir=?\n" \
        " --file=?\n" \
        "   indir: a directory containing input files to be processed\n" \
        "   outdir: a directory containing output files\n" \
        "   file: a single file to process\n")

# convert csv file to raw
def csv2raw(infile,outdir):
  try:
    f = open(infile,'r')
  except IOError:
    print("Can't open file: " + infile)
    return

  infile = infile.replace('\\','/')

  # parse the subject name from the file name
# base = infile[:infile.rfind('.')]
  base = os.path.basename(infile)
  print("Processing: ", infile, "[", base, "]")

  # strip out extension
  filename, ext = os.path.splitext(base)
  print('Processing: ' + filename)

  subj = filename
  print('subj: ', subj)

  outfile = None
  outfile = open(outdir + subj + '.raw','w+')
  print('Outfile: %s' % (outdir + subj + '.raw'))

  # read lines, throwing away first one (header)
  linelist = f.read().splitlines()
  header = linelist[0].split(',')
  linelist = linelist[1:]

  for idx, label in enumerate(header):
    if "seconds" in label.strip():
      TIME = idx
    if "gaze_pos_x" == label.strip():
      BPOGX = idx
    if "gaze_pos_y" == label.strip():
      BPOGY = idx
    if "gaze_pos_val" == label.strip():
      VALID = idx

  # reset coords
  x = ''
  y = ''
  t = ''

  # process each line, splitting on ','
  for line in linelist:
    elements = line.split(',')
    entry = np.asarray(elements)

    # dump out gaze coordinates
    if(outfile is not None and
       entry[BPOGX] != '' and
       entry[BPOGY] != '' and
       entry[VALID] != '' and
       entry[BPOGX] != x and
       entry[BPOGY] != y and
       entry[TIME] != t):

       # data already normalized, no need to divide by screen size
       x = str(float(entry[BPOGX]))
       y = str(float(entry[BPOGY]))
       t = entry[TIME]
       # Tobii Glasses 2 validity should be 0.0 for everything OK,
       # non-zero means problem
       v = float(entry[VALID])

#      print "x,y,t,v: ", x,y,t,v

       # don't process empty or negative coords
#      if locale.atof(x) < 0 or locale.atof(y) < 0:
#        print "locale data: ",locale.atof(x),locale.atof(y),"*******"
#      else:
#        print "locale data: ",locale.atof(x),locale.atof(y)
       if(v < 0.1 and x != '' and y != '' and \
          locale.atof(x) < 1 and locale.atof(y) < 1 and \
          locale.atof(x) >= 0 and locale.atof(y) >= 0):
         strout = "%f %f %f" % (locale.atof(x),locale.atof(y),float(t))
#        strout = "%f %f %f" % (locale.atof(x),1-locale.atof(y),float(t))
         outfile.write(strout + '\n')

  outfile.close()

def main(argv):
# if not len(argv):
#   usage()

  try:
    opts, args = getopt.getopt(argv, '', \
                 ['indir=','outdir=','file=',\
                  'hertz=','sfdegree=','sfcutoff='])
  except getopt.GetoptError as err:
    usage()
    exit()

  file = None

  for opt,arg in opts:
    opt = opt.lower()
    if(opt != '--file'):
      arg = arg.lower()

    if opt == '--indir':
      indir = arg
    elif opt == '--outdir':
      outdir = arg
    else:
      sys.argv[1:]

  # get .csv input files to process
  if os.path.isdir(indir):
    files = glob.glob('%s/*.csv' % (indir))

  # if user specified --file="..." then we use that as the only one to process
  if(file != None and os.path.isfile(file)):
    files = [file]

  for file in files:
    csv2raw(file,outdir)

#######################
#locale.setlocale( locale.LC_ALL, 'en_US.UTF-8' )

if __name__ == "__main__":
  main(sys.argv[1:])
