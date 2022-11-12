#!/usr/bin/env python3

import sys,os,getopt,glob
import json
import locale
import numpy as np
import collections
import pprint

def usage():
  print("Usage: python card2json.py " \
        " --indir=? -outdir=?\n" \
        " --file=?\n" \
        "   indir: projects/ directory containing input files off card\n" \
        "   outdir: a directory containing output files\n" \
        "   file: a single file to process\n")

# convert csv file to raw
def card2json(infile,outdir):
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
                 ['indir=','outdir=','outid=','file=',\
                  'hertz=','sfdegree=','sfcutoff='])
  except getopt.GetoptError as err:
    usage()
    exit()

  outdir = "./"
  outid = ""
  file = None

  for opt,arg in opts:
    opt = opt.lower()
    if(opt != '--file'):
      arg = arg.lower()

    if opt == '--indir':
      indir = arg
    elif opt == '--outdir':
      outdir = arg
    elif opt == '--outid':
      outid = arg
    else:
      sys.argv[1:]

  if outid == "":
    print("failed to find outid in args. AKA failed json generation. Exiting()")
    exit()

  files = []

  info = {}

  print("indir: ", indir)
  print("outdir: ", outdir)
  print("outid: ", outid)

  # get paths to dirs that are in the projects/<project>/recordings/ dir
  if os.path.isdir(indir):
    for top, dirs, listing in os.walk(indir):
      base = os.path.basename(top)

      print("top: ", top)
      print("dirs: ", dirs)
      print("listing: ", listing)
      print("base: %s\n" % (base))

      # we're at project level, add dir list to 'projects' key
      if base == "projects":
        pass

      elif 'project.json' in listing:
        pr_id = base
        prj_path = os.path.join(top,'project.json')
        with open(prj_path) as f:
          prj_dict = json.load(f)
#         print "prj_dict: ", prj_dict

          info[pr_id] = prj_dict
          info[pr_id]['id'] = outid
          info[pr_id]['recordings'] = []

#         print "info:\n", pprint.pprint(info)
          print("info:\n", json.dumps(info, sort_keys=True, indent=2))
#         print >> outfile, json.dumps(info, sort_keys=True, indent=2)

      # we're at recordings level, add dir list to 'recordings' key 
      elif base == "recordings":
        pass

#       pr_id = top.split('/')[-2]
#       print "pr_id: ", pr_id
#       info[pr_id]['recordings'] = dirs

      # we're at a recording level, need to parse recording.json
      elif 'recording.json' in listing:

        # parse 'recording.json' file to get recording name
        rec_path = os.path.join(top,'recording.json')
        if os.path.isfile(rec_path):
          print("rec_path: ", rec_path) 
          with open(rec_path) as f:
            rec_dict = json.load(f)
            print("rec_dict['rec_id']: ", rec_dict['rec_id'])
            print("rec_dict['rec_project']: ", rec_dict['rec_project'])
            print("rec_dict['rec_info']['Name']: ", rec_dict['rec_info']['Name'])
            # set up conversion dict entry, e.g., {"Recording116": "lakol3b"}
            con_dict = {}
            con_dict[rec_dict['rec_info']['Name']] = rec_dict['rec_id']
            print("con_dict: ", json.dumps(con_dict))
            pr_id = rec_dict['rec_project']
#           if info[pr_id].has_key('recordings'):
#             info[pr_id]['recordings'].append(rec_dict)
#           else:
#             info[pr_id]['recordings'] = rec_dict
            if 'recordings' in info[pr_id]:
              info[pr_id]['recordings'].append(con_dict)
            else:
              info[pr_id]['recordings'] = con_dict

#         print "info:\n", pprint.pprint(info)
          print("info:\n", json.dumps(info, sort_keys=True, indent=2))
#         print >> outfile, json.dumps(info, sort_keys=True, indent=2)

      else:
        pass
#       print "top: ", top
#       print "dirs: ", dirs
#       print "listing: ", listing
#       print "base: ", base

  # if user specified --file="..." then we use that as the only one to process
  if(file != None and os.path.isfile(file)):
    files = [file]

  for file in files:
    print("file: ", file) 

  outfile = None
  outfile = open(outdir + 'generated.json','w')
  print('Outfile: %s' % (outdir + 'generated.json'))
  print(json.dumps(info, sort_keys=True, indent=2), file=outfile)

  outfile.close()

#######################
#locale.setlocale( locale.LC_ALL, 'en_US.UTF-8' )

if __name__ == "__main__":
  main(sys.argv[1:])
