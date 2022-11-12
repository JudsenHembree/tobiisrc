#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Base this code to a certain extent on trek/src/bike/vidgp.py

import platform, sys, os, math
import locale
import getopt
import glob
import numpy as np
import cv2
import cv2.aruco as aruco
import datetime as dt

from threading import Thread

from PIL import Image

from tqdm import tqdm

# local includes
from scanpath import Scanpath
from aoi import AOI
from dynaoi import dynAOI
from arucoaoi import ArucoAOI
from bw import Butterworth as BW
from sg import SG

def printProgressBar(iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█'):
  """
  Call in a loop to create terminal progress bar
  @params:
      iteration   - Required  : current iteration (Int)
      total       - Required  : total iterations (Int)
      prefix      - Optional  : prefix string (Str)
      suffix      - Optional  : suffix string (Str)
      decimals    - Optional  : positive number of decimals in percent complete (Int)
      length      - Optional  : character length of bar (Int)
      fill        - Optional  : bar fill character (Str)
  """
  percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
  filledLength = int(length * iteration // total)
  bar = fill * filledLength + '-' * (length - filledLength)
  print('%s |%s| %s%% %s' % (prefix, bar, percent, suffix))
  for i in range(length):
    print('\b')
  # Print New Line on Complete
  if iteration == total: 
      print('\n')

def setfk(fk,points,vts):

  # debug
# print "setfk: vts = ", vts
# print "setfk: fk = ", fk
# print "setfk: points[0].t = ", points[0].t

  if len(points) > 0:

    # use time relative to beginning (synching scanpath to video stream)
#   while fk < len(points)-1 and (points[fk].t - points[0].t) < vts:
#   while fk < len(points)-1 and points[fk].t < vts:
    while fk < len(points)-1 and points[fk].t + points[fk].duration < vts:
#   while fk < len(points)-1 and mt < vts:
#     print "vts, points.t, fk: ", vts, (points[fk].t - points[0].t), fk
      fk += 1

  return fk

def setpk(pk,points,vts):

  if len(points) > 0:

    # use time relative to beginning (synching scanpath to video stream)
#   while pk < len(points)-1 and (points[pk].t - points[0].t) < vts:
    while pk < len(points)-1 and points[pk].t < vts:
#     print("vts, points.t, pk: ", vts, (points[pk].t - points[0].t), pk)
      pk += 1

  return pk

def usage(argv):
  print("Usage: ", argv[0], "\n" \
        " --output\n" \
        " --width=? --height=? --screen=? --dist=?\n" \
        " --indir=? --viddir=? --outdir=?\n" \
        " --vfile=?\n" \
        " --file=?\n" \
        " --herz=?\n"\
        " --sfdegree=? --sfcutoff=?\n"\
        " --dfdegree=? --dfwidth=?\n"\
        " --vt=?\n" \
        "   width, height: screen dimensions (in pixels)\n" \
        "   screen: screen diagonal dimension (in inches)\n" \
        "   dist: viewing distance (in inches)\n" \
        "   indir: a directory containing input files to be processed\n" \
        "   viddir: a directory containing image files\n" \
        "   outdir: a directory containing output files\n" \
        "   vfile: video input file\n" \
        "   file: (raw) gaze input file\n" \
        "   herz: the sampling rate of the data\n" \
        "   sfdegree: butterworth filter smoothing degree \n" \
        "   sfcutoff: butterworth filter smoothing cutoff \n" \
        "   dfdegree: savitzky-golay filter degree \n" \
        "   dfwidth: savitzky-golay filter width \n" \
        "   vt: min velocity for change to be a saccade\n"\
        "   output: if present output video\n")

def main(argv):

  # get args
  try:
    opts, args = getopt.getopt(argv[1:], '', \
                 ['width=', 'height=',\
                  'herz=',\
                  'screen=', 'dist=',\
                  'indir=', 'viddir=', 'outdir=',\
                  'file=', 'sfdegree=', 'sfcutoff=',\
                  'dfdegree=', 'dfwidth=',\
                  'vt=', 'aoi=',\
                  'vfile=', 'file=', 'output'])

  except getopt.GetoptError as err:
    print("getopt error: ", err)
    usage(argv)
    sys.exit()

  vfile = None
  file = None
  outfile = None

  scanpath = None

  output = False

  width = 640
  height = 360
  herz = 100
  indir = './data/raw/'
  outdir = './data/'
  viddir = '../../data/risk_v_contrast/'
  sfdegree = 2
  sfcutoff = 6.15 # less smooth
  dfo = 1
  dfwidth = 3
  dfdegree = 3
  smooth = False
  # narrow-angle lens is 60 deg field of view
  # 1280 x 720, r = 1468.60 diagonal
  # r = 2D tan(theta/2) = 2(31.50) * tan(30) = 36 inches
# screen = 36
  screen = 18
# dist = 23.62
  dist = 31.50
# T = 20
  T = 10

  # guess as to how to squish video fps
# tdilation = 2.0
  tdilation = 29.8424492273

  rvec_u = None
  rvec_k = 0

  origin = None
  x_axis = None
  y_axis = None
  z_axis = None

  # camera paremters for pose estimation (from Seva's file)
  # see: http://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  # centimeters?  docs say in meters
# markerLength = 100.0
# markerLength = 1.0
# markerLength = 10.0
  markerLength = 0.01
# markerLength = 0.1
  # this is rather important
# fx, fy = 486.181, 2639.31
# cx, cy = -131.322, 20.8217
# fx, fy = 1.0, 1.0
# cx, cy = 0.0, 0.0
  # guessing
  # for info on intrinsic matrix,
  # see also http://ksimek.github.io/2013/08/13/intrinsic/
  # and for camera calibration:
  # https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
# fx, fy = 10000.0, 10000.0
# cx, cy = 0.0, 0.0
  # see: https://github.com/pupil-labs/pupil/issues/1316
# fx, fy = 1120.43099, 1077.34094
# cx, cy = 968.35635, 545.695766
  # Clemson calibration using 13" lapotp
# fx, fy = 1099.67068,  1105.88290
# cx, cy =  955.43772, 528.128416
  # UTRGV
  fx, fy = 1011.96491,  1005.40935
  cx, cy =  929.78932, 528.810188
  # 3x3 matrix
# cameraMatrix = np.float32([[fx,0.0,cx],[fy,0.0,cy],[0.0,0.0,1.0]])
  cameraMatrix = np.array([ [  fx, 0.0,  cx],\
                            [ 0.0,  fy,  cy],\
                            [ 0.0, 0.0, 1.0] ])
# k1, k2, k3 = -0.0683092, -0.298703, 0.103089
# p1, p2 = 0.00189006, 0.0582983
# k1, k2, k3 = 0.0, 0.0, 0.0
# p1, p2 = 0.0, 0.0
  # Clemson calibration
# k1, k2, k3 = 0.09961660299292627, -0.21847900301383041, 0.09417837101183982
# p1, p2 = -0.0010681464641609897, -0.0014568525518904656
  # UTRGV
  k1, k2, k3 = 0.1308405, -0.49590051, 0.52786418
  p1, p2 = -0.00202674, -0.00609927
  # Pupil Labs
# k1, k2, k3 = -0.0221333859, 0.721243070, -3.66524347
# p1, p2 = 0.00406219785, -0.00203063092
  # 1x5 vector
# distCoeffs = np.float32([[k1,k2,p1,p2]])
  distCoeffs = np.array([[k1,k2,p1,p2,k3]])

  # xoffset, yoffset, w, h
  # for marker 0
  privacyText0 = ArucoAOI(140.0, -170.0,10.0, 2.0, markerLength, "privacyText0")
  privacyIcon0 = ArucoAOI(550.0,   30.0, 1.0, 1.0, markerLength, "privacyIcon0")
  # for marker 1
  privacyText1 = ArucoAOI(140.0, -130.0,10.0, 2.0, markerLength, "privacyText1")
  privacyIcon1 = ArucoAOI(550.0,   80.0, 1.0, 1.0, markerLength, "privacyIcon1")
  # for marker 2
  privacyText2 = ArucoAOI(140.0, -170.0,10.0, 2.0, markerLength, "privacyText2")
  privacyIcon2 = ArucoAOI(550.0,   30.0, 1.0, 1.0, markerLength, "privacyIcon2")
  # for marker 3
  privacyText3 = ArucoAOI(140.0, -130.0,10.0, 2.0, markerLength, "privacyText3")
  privacyIcon3 = ArucoAOI(550.0,   80.0, 1.0, 1.0, markerLength, "privacyIcon3")

  readmorText = ArucoAOI(140.0, -340.0, 9.5, 9.0, markerLength, "readmorText")

  # see: https://docs.opencv.org/3.1.0/d9/d6a/group__aruco.html
# aruco_dict_org = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
  aruco_dict_org = aruco.Dictionary_get(aruco.DICT_4X4_50)
  aruco_parameters = aruco.DetectorParameters_create()

  aruco_parameters.minMarkerPerimeterRate = 0.02 # default 0.03
  aruco_parameters.maxMarkerPerimeterRate = 1.00 # default 4.0
  aruco_parameters.polygonalApproxAccuracyRate = 0.02 # default 0.05
  aruco_parameters.minCornerDistanceRate = 0.02 # default 0.05
  aruco_parameters.minMarkerDistanceRate = 0.02 # default 0.05
  aruco_parameters.minDistanceToBorder = 3 # default 3
  aruco_parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13 # default 0.13
  aruco_parameters.maxErroneousBitsInBorderRate = 0.35 # default 0.35
  aruco_parameters.markerBorderBits = 1 # default 1
  aruco_parameters.minOtsuStdDev = 5.0 # default 5.0
  aruco_parameters.errorCorrectionRate = 0.6 # default 0.6
# aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
  aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
  aruco_parameters.cornerRefinementMinAccuracy = 0.1 # default is 0.1
  aruco_parameters.cornerRefinementMaxIterations = 30 # default is 30
  aruco_parameters.cornerRefinementWinSize = 2 # default is 5

  # process args
  for opt,arg in opts:
    if opt == '--output':
      output = True
    elif opt == '--vfile':
      vfile = arg
    elif opt == '--file':
      file = arg
    elif opt == '--indir':
      indir = arg
    elif opt == '--viddir':
      viddir = arg
    elif opt == '--outdir':
      outdir = arg
    elif opt == '--pltdir':
      pltdir = arg
    elif opt == '--width':
      width = int(arg)
    elif opt == '--height':
      height = int(arg)
    elif opt == '--screen':
      screen = float(arg)
    elif opt == '--dist':
      dist = float(arg)
    elif opt == '--file':
      file = arg
    elif opt == '--herz':
      herz = float(arg)
    elif opt == '--sfdegree':
      sfdegree = float(arg)
    elif opt == '--sfcutoff':
      sfcutoff = float(arg)
    elif opt == '--dfdegree':
      dfdegree = float(arg)
    elif opt == '--dfwidth':
      dfwidth = float(arg)
    elif opt == '--vt':
      T = float(arg)
    elif opt == '--aoi':
      aoifile = arg
    else:
      sys.argv[1:]

  if output:
    print("Outputting to 'output.mov'")

  # get .raw input files to process
  if os.path.isdir(indir):
    files = glob.glob('%s/*.raw' % (indir))

  # if user specified --file="..." then we use that as the only one to process
  if(file != None and os.path.isfile(file)):
    files = [file]

  for file in files:

    if file is not None and os.path.isfile(file):
      base = os.path.basename(file)
      print("Processing: ", file, "[", base, "]")
      filename, ext = os.path.splitext(base)

      subj = filename
#     this is for f2f
#     subj = filename.split('_')[0]
#     trial = filename.split('_')[1]
#     vfile = viddir + subj + '/' + trial + '/' + 'world.mp4'
#     this is for risk_v_contrast
      vfile = viddir + subj + '.mp4'

      print("subj, vfile: ", subj, vfile)

      # reset vars
      if outfile is not None:
        outfile.close()
      outfilename = "%s/%s-vfpt%s" % (outdir,filename,".dat")

      # checkpoint: if the output file already exists, skip processing this file
      if os.path.isfile(outfilename):
        continue

      outfile = open(outfilename,'w')
      outfile.write("frame,timestamp,aoi,dur" + '\n')

      cap = None
      frame = None
      frameidx = 0
      output = False
      fk = 0
      pk = 0
      vtd = dt.timedelta()
      vts = 0.0

      if scanpath:
        del scanpath

      scanpath = Scanpath()
      scanpath.parseFile(file,width,height,herz)

#     oherz = 1.0/scanpath.getsamplingrate()
#     print "Observed samping rate: ", oherz
      print("Applied samping rate: ", herz)
      print("Observed duration: ", scanpath.gettotalduration())
      print("Observed number of gaze points: ", scanpath.getnumgpoints())
      oherz = float(scanpath.getnumgpoints())/float(scanpath.gettotalduration())
      print("Observed sampling rate: ", oherz)

      base = os.path.basename(file)
      print("Processing: ", file, "[", base, "]")

      # split filename from extension
      filename, ext = os.path.splitext(base)

      scanpath.smooth("%s/%s-smth%s" % (outdir,filename,".dat"),\
                      width,height,oherz,sfdegree,sfcutoff, smooth)
      scanpath.differentiate("%s/%s-diff%s" % (outdir,filename,".dat"),\
                      width,height,screen,dist,oherz,dfwidth,dfdegree,dfo)
      scanpath.threshold("%s/%s-fxtn%s" % (outdir,filename,".dat"),\
                      width,height,T)
      scanpath.amfoc("%s/%s-amfo%s" % (outdir,filename,".dat"),\
                      width,height)
#     scanpath.gridify("%s/%s-aois%s" % (outdir,filename,".csv"),\
#                     subj,cond,width,height,xtiles,ytiles)

      scanpath.dumpDAT("%s/%s%s" % (outdir,filename,".dat"),width,height)

      # old bit of code back when we used 'points' to point to either list of
      # gazepoints or fixations
#     points = scanpath.gazepoints
      points = scanpath.fixations
#     if we have frame index in gaze data
#     for fi, pt in enumerate(points):
#       print "fixation [%d] (%f, %f, %f) frame [%d:%d]" % \
#             (fi, pt.x, pt.y, pt.duration, pt.sf, pt.ef)
#     if we don't have frame index in gaze data
      for fi, pt in enumerate(points):
        print("fixation [%d] (%f, %f, %f) " % \
              (fi, pt.x, pt.y, pt.duration))

      # open video file
      if vfile is not None and os.path.isfile(vfile):
        print("Opening video file: ", vfile)
        # grab video from file
        cap = cv2.VideoCapture(vfile)
      else:
        # grab video from available camera (e.g., laptop cam)
        cap = cv2.VideoCapture(0)

      size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), \
              int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

      print("size: ", size)
      fps = cap.get(cv2.CAP_PROP_FPS)
      print("fps: ", fps)
      frame_rate = 1./fps
      print("frame_rate: ", frame_rate)
      frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
      print("frames: ", frames)
      print("total time: ", frame_rate * frames)
      # scanpath duration should indicate how long the video is
      # incomding fps of the video is probably incorrect (e.g., 60 fps)
      # so we need to rescale the time duration of the video to the duration
      # of the scanpath, this is:
      #    (float(scanpath.gettotalduration()) / (frame_rate * frames))
      # fps divided by the above gives us the desired fps
      if scanpath is None:
        print("No scanpath: did you run 'make raw'?")
        cap.release()
        sys.exit()
      else:
        tdilation = \
          fps / (float(scanpath.gettotalduration()) / (frame_rate * frames))
        print("Observed sampling rate: ", tdilation)
        print("desired frame rate and total time: ", \
            (fps/tdilation)*frame_rate, (fps/tdilation)*frame_rate*frames)

      # resize video by (.5,.5)
      size = (size[0]/2, size[1]/2)
      if size[0] != width or size[1] != height:
        print("WARNING: frame and scanpath size mismatch!")
        size = (width, height)
      print("size, fps: ", size, fps)
      cap.set(size[0],size[1])

#     pbar = tqdm(total=len(scanpath.fixations))
      pbar = tqdm(total=frames)

      # the main loop, was beginning of idle(), now we iterate over fixations
      # this is only if we're doing it fixation by fixation, i.e., we have
      # frame index info at each gaze point (like Pupil Labs does it)
#     for fi, pt in enumerate(points):
#
#       cap.set(cv2.CAP_PROP_POS_FRAMES,pt.sf)
#
      # the main loop, frame by frame (no frame index per gaze point)
      while(cap.isOpened()):

        # capture frame-by-frame
        ret, frame = cap.read()

        if ret is True:

#         print "frame %5d" % (cap.get(cv2.CAP_PROP_POS_FRAMES))
          frameidx = cap.get(cv2.CAP_PROP_POS_FRAMES)
 
          # resize frame read in
          frame = cv2.resize(frame, size, 0, 0, cv2.INTER_CUBIC)
 
          # get frame time stamp (in milliseconds)
          ms = cap.get(cv2.CAP_PROP_POS_MSEC) * (fps/tdilation)
          # rescale into seconds
          vts = ms/1000.0
          # represent ms timestamp as datetime.timedelta (don't += here!)
          vtd = dt.timedelta(milliseconds=ms)
#         print "********************** TIMESTAMP: ", ms

          # our operations on the frame come here
          gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

          #################### ARUCO ########################
          origin = None
          x_axis = None
          y_axis = None
          z_axis = None
 
          ids_seen_already = []
 
          # aruco marker detection
          corners_org, ids_org, rejectedImgPoints_org = \
           aruco.detectMarkers(frame,aruco_dict_org,parameters=aruco_parameters)
#         print "CORNERS_ORG:"
#         print(corners_org)

          # note: this will draw false positives as well (annoying)
#         frame = aruco.drawDetectedMarkers(frame,corners_org)
          frame = aruco.drawDetectedMarkers(frame,corners_org,ids_org)

          # pose estimation
          # opencv v3.4.1.2
          rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners_org,\
                                                      markerLength,\
                                                      cameraMatrix,\
                                                      distCoeffs)

          # I think I see the problem: need to process for each marker!
          if rvec is not None and tvec is not None:
            for i in range(len(ids_org)):
#           print "ids_org = ", ids_org, "(", len(ids_org), ")"
              # skip id 0, seems like false positive
#             if ids_org[i] == 0:
#               continue
              # in this video we only use ids 0, 1, 2, 3, 4
              if ids_org[i] < 0 or ids_org[i] > 4:
                continue
              # skip duplicates
              if ids_org[i] in ids_seen_already:
                continue
              ids_seen_already.append(ids_org[i])
  
              if ids_org[i] == 0:
                privacyText0.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
                privacyIcon0.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
              elif ids_org[i] == 1:
                privacyText1.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
                privacyIcon1.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
              elif ids_org[i] == 2:
                privacyText2.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
                privacyIcon2.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
              elif ids_org[i] == 3:
                privacyText3.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
                privacyIcon3.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
              elif ids_org[i] == 4:
                readmorText.project(rvec[i],tvec[i],cameraMatrix,distCoeffs)
              elif ids_org[i] > 4:
                print("UNKNOWN MARKER")

          del ids_seen_already[:]

          #################### ARUCO ########################

          # display the resulting frame
#         cv2.imshow('frame',gray)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#           break

#         print "beginning old display() routine"
          # begin old display()

          gp = None

          # advanced pk to current gaze point
          if scanpath is not None:
            fk = setfk(fk,scanpath.fixations,vts)
            pk = setpk(pk,scanpath.gazepoints,vts)

            # get the "current" gaze point (need this for AOI testing below)
            # y-flip
#           gp = (scanpath.gazepoints[pk].x, 1.0 - scanpath.gazepoints[pk].y) \
#                if 0 < pk and pk < len(scanpath.gazepoints) else None
#           gp = (scanpath.fixations[fk].x, 1.0 - scanpath.fixations[fk].y) \
#                if 0 < fk and fk < len(scanpath.fixations)-1 else None
            gp = (scanpath.fixations[fk].x, \
            1.0 - scanpath.fixations[fk].y, \
                  scanpath.fixations[fk].getDuration()) \
                 if 0 < fk and fk < len(scanpath.fixations)-1 else None
#           if gp is not None:
#             print "gp: (%f, %f)" % (gp[0],gp[1])
#           else:
#             print "gp: None"

          # get the "current" gaze point (need this for AOI testing below)
          # this is only if we have frame index data per gp
          # y-flip
#         gp = (pt.x, 1.0 - pt.y, pt.getDuration())
#         if gp is not None:
#           print "gp: (%f, %f)" % (gp[0],gp[1])
#         else:
#           print "gp: None"

          # debugging
#         if gp is not None:
#           gdot = (int(gp[0]*width),int(height - gp[1]*height))
##          print "(%d,%d)" % (gdot[0],gdot[1])
#           frame = cv2.circle(frame,gdot,5,(0,0,255,128),-1)
#         cv2.imshow('frame',frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#           break
 
          # old draw faces
#         print str(vtd).replace('.',',')
          aoihit = False
          hitstr = ''

          if gp is not None:

            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 0 in ids_org and \
               privacyText0.inside(gp[0]*width,height - gp[1]*height):
#            print "got hit on privacyText: %d" % (0)
              hitstr = hitstr + 'privacyText_' + str(0)
              aoihit = True
            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 0 in ids_org and \
               privacyIcon0.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyIcon: %d" % (0)
              hitstr = hitstr + 'privacyIcon_' + str(0)
              aoihit = True

            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 1 in ids_org and \
               privacyText1.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyText: %d" % (1)
              hitstr = hitstr + 'privacyText_' + str(1)
              aoihit = True
            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 1 in ids_org and \
               privacyIcon1.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyIcon: %d" % (1)
              hitstr = hitstr + 'privacyIcon_' + str(1)
              aoihit = True

            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 2 in ids_org and \
               privacyText2.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyText: %d" % (2)
              hitstr = hitstr + 'privacyText_' + str(2)
              aoihit = True
            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 2 in ids_org and \
               privacyIcon2.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyIcon: %d" % (2)
              hitstr = hitstr + 'privacyIcon_' + str(2)
              aoihit = True

            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 3 in ids_org and \
               privacyText3.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyText: %d" % (3)
              hitstr = hitstr + 'privacyText_' + str(3)
              aoihit = True
            # need to scale gp coords and do y-flip for inside test
            if ids_org is not None and 3 in ids_org and \
               privacyIcon3.inside(gp[0]*width,height - gp[1]*height):
#             print "got hit on privacyIcon: %d" % (3)
              hitstr = hitstr + 'privacyIcon_' + str(3)
              aoihit = True

            # append gp duration if we had hits
            if not aoihit:
              hitstr = hitstr + 'noaoi'
            hitstr = hitstr + ",%f" % (gp[2])

          else:
            hitstr = ",,"

          # video timestamp text (bitmap)
#         hmsstr = str(vtd).replace('.',',')
#         hmsstr = str(vtd)
          # use milliseconds instead of HH:MM:SS
          hmsstr = str(vts)
   
          infostr = "%d,%s,%s" % (frameidx,hmsstr,hitstr)
#         print infostr
          outfile.write(infostr + '\n')

          # this is only if are enumerating by gaze points
#         ret, frame = cap.read()
#         print "frame %5d" % (cap.get(cv2.CAP_PROP_POS_FRAMES))
#         frameidx = cap.get(cv2.CAP_PROP_POS_FRAMES)
   
#         printProgressBar(frameidx, frames, prefix = 'Progress:', suffix = 'Complete', length = 40)
          pbar.update(1)
        else: # ret is False
          pbar.close()
          cap.release()
      # end while (set of frames for this fixation)

    else:
      print("Error opening: ", file)

  # end for files

  sys.exit()

#######################
# on Windows, see bottom of this web page:
# http://stackoverflow.com/questions/19709026/how-can-i-list-all-available-windows-locales-in-python-console

#locale.setlocale( locale.LC_ALL, 'en_US.UTF-8' )
#locale.setlocale( locale.LC_ALL, 'de_DE' )
if platform.system() == 'Windows':
  locale.setlocale( locale.LC_ALL, 'German' )
else:
  locale.setlocale( locale.LC_ALL, 'de_DE' )

if __name__ == '__main__':
# main(sys.argv[1:])
  main(sys.argv)
