#!/usr/bin/env python

import platform, sys, os, math
import locale
import getopt
import numpy as np
import cv2
import datetime as dt

from threading import Thread

from PIL import Image

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# local includes
from scanpath import Scanpath
from aoi import AOI
from dynaoi import dynAOI
from bw import Butterworth as BW
from sg import SG

class Point:
  def __init__(self,x,y,t):
    self.x = x
    self.y = y
    self.t = t

global fps
global herz
global tdilation
global vtd
global vts
global cap
global frame
global findx
global size
global out
global output
global texName
global vidpause

global fk
global pk
#global points
global bw_filter_x, bw_filter_y, sg_filter_x, sg_filter_y
global scanpath

def usage(argv):
  print("Usage: ", argv[0], "\n" \
        "  --vfile=?\n" \
        "  --file=?\n" \
        "  --output\n" \
        "    vfile: video input file\n" \
        "    file: (raw) gaze input file\n" \
        "    output: if present output video\n")

def glDrawCircle(radius,slices):

# glBegin(GL_POLYGON)
# for v in xrange(0, slices):
#   angle  = float(v) * 2.0 * np.pi / slices
#   glVertex2f(np.cos(angle)*radius, np.sin(angle)*radius)
# glEnd()

  thickness = 2

  quadric = gluNewQuadric()
  gluDisk(quadric,radius-thickness,radius,slices,1)
  del quadric

def glDrawDisc(radius,slices):

# glBegin(GL_POLYGON)
# for v in xrange(0, slices):
#   angle  = float(v) * 2.0 * np.pi / slices
#   glVertex2f(np.cos(angle)*radius, np.sin(angle)*radius)
# glEnd()

  quadric = gluNewQuadric()
  gluDisk(quadric,0,radius,slices,1)
  del quadric

def gaze_stream(x,y,dt,ww,wh):

  global sg_filter_x
  global sg_filter_y

  # dt should be 0.016 (s)
  if dt <= 0.0:
    return False

  # gaze velocity in screen coordinates
  dx = sg_filter_x.sgf(x*float(ww))
  dy = sg_filter_y.sgf(y*float(wh))

  # sampling period in s
# period = float(float(dt)/1000.0)
  period = float(float(dt))
  # 3 is dfwidth
  dt = period * float(3)

  # window width, height
  screen = 27
  r = math.sqrt(float(ww)*float(ww) + float(wh)*float(wh))
  dpi = r/float(screen)

  # viewing distance (in)
  D = float(23.62)

# fov = 2*math.degrees(math.atan(math.radians(screen/(2*D))))
  fov = 2*math.degrees(math.atan2(screen,2*D))
  fovx = 2*math.degrees(math.atan2(float(ww)/dpi,2*D))
  fovy = 2*math.degrees(math.atan2(float(wh)/dpi,2*D))

# print("screen subtends %f (%f x %f) degrees" % \
#                        (float(fov),float(fovx),float(fovy)))
# print("screen aspect = %f" % float(float(ww)/float(wh)))
# print("sampling period = %f (ms)" % float(period * 1000.0))
# print("filter window = %f (ms)" % float(dt * 1000.0))
# print("dpi = %f" % dpi)

  degx = 2*math.degrees(math.atan2((dx/dpi),(2*D)))
  degy = 2*math.degrees(math.atan2((dy/dpi),(2*D)))

  # degx, degy is degrees per filter window, div by dt to get per second
  velx = degx / dt
  vely = degy / dt

  vel = math.sqrt(float(velx)*float(velx) + float(vely)*float(vely))

# print("gaze velocity (pixels) = (%d,%d)" % (dx, dy))
# print("gaze velocity (deg) = (%d,%d)" % (velx,vely))
# print("gaze velocity (deg) = (%f)" % (vel))

  # test against threshold (deg/s)
  if vel > 20:
#   print("fixating: NO")
    return False
  else:
#   print "fixating: YES"
    return True

def setfk(points):

  global fk
  global vts

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

def setpk(points):

  global pk
  global vts

  if len(points) > 0:

    # use time relative to beginning (synching scanpath to video stream)
#   while pk < len(points)-1 and (points[pk].t - points[0].t) < vts:
    while pk < len(points)-1 and points[pk].t < vts:
#     print "vts, points.t, pk: ", vts, (points[pk].t - points[0].t), pk
      pk += 1

def renderFixations(width, height, gazewin, points):

  global vts
  global fk

  # diagnoal
  d = math.sqrt(float(width)*float(width) + float(height)*float(height))

  if len(points) > 0:

    # for display purposes: use dk as look back in time
    dk = fk
    dur = 0.
#   while dk > 1 and (vts - gazewin) <= (points[dk].t - points[0].t):
#   while dk > 1 and (vts - gazewin) <= points[dk].t \
#                and points[dk].t + points[dk].duration <= (vts + gazewin):
    while dk > 1 and points[dk].t - gazewin <= vts and vts <= \
                     points[dk].t + points[dk].duration + gazewin:
      # point is on this frame, draw it
#     print "!!!!!!!!! got point", points[dk].x, " , ", points[dk].y
      dt = points[dk].t - points[dk-1].t if dk-1 >= 0 else points[dk].t
      dur += dt

      t = max(1.0 - dur/gazewin,0.5)

      glColor4f(241./256., 163./256., 64./256., t)

#     glBegin(GL_POINTS)
#     glVertex2f(points[dk].x*width, points[dk].y*height)
#     glEnd()

      glPushMatrix()
      # y-flip
      glTranslatef(points[dk].x*width, (1. - points[dk].y)*height, 0.0)
#     glDrawCircle(30,36)
      glDrawCircle(points[dk].getPercentDuration() * d/6.0,18)
      glPopMatrix()

      dk -= 1
  
def renderPoints2D(width, height, gazewin, points):

  global vts
  global pk

  if len(points) > 0:

    # for display purposes: use dk as look back in time
    dk = pk
    dur = 0
#   while dk > 1 and (vts - gazewin) <= (points[dk].t - points[0].t):
    while dk > 1 and (vts - gazewin) <= points[dk].t:
        # point is on this frame, draw it
#       print "!!!!!!!!! got point", points[dk].x, " , ", points[dk].y
        dt = points[dk].t - points[dk-1].t if dk-1 >= 0 else points[dk].t
        dur += dt

        t = max(1.0 - dur/gazewin,0.5)

#       print "t, dt, dur, gazewin: ", t, dt, dur, gazewin
        glColor4f(0.7, 0.7, 0.7, t)

#       glBegin(GL_POINTS)
#       glVertex2f(points[dk].x*width, points[dk].y*height)
#       glEnd()

        x, y = points[dk].x, points[dk].y
        xp, yp = points[dk-1].x, points[dk-1].y
        if x > 0.1 and y > 0.1 and xp > 0.1 and yp > 0.1:

          glPushMatrix()
          # y-flip
          glTranslatef(points[dk].x*width, (1. - points[dk].y)*height, 0.0)
          glDrawDisc(3,36)
          glPopMatrix()

          glBegin(GL_LINES)
          # y-flip
          glVertex2f(points[dk].x*width, (1. - points[dk].y)*height)
          glVertex2f(points[dk-1].x*width, (1. - points[dk-1].y)*height)
          glEnd()
  
        dk -= 1

def reshape(width, height):
  glViewport(0, 0, width, height)

  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()

  # l, r, b, t, near, far
# glOrtho(-width/height, width/height, -1.0, 1.0, -1.0, 1.0)
  # (0,0) at lower left
  glOrtho(0, width, 0, height, -1.0, 1.0)
  # (0,0) at upper left
# glOrtho(0, width, height, 0.0,  0.0, 1.0)
# glOrtho(0, width, height, 0.0, -1.0, 1.0)
# gluOrtho2D(0, width, 0, height)

  glMatrixMode(GL_MODELVIEW)
  glLoadIdentity()

def keyboard(key, x, y):

  global vidpause

  if key is '\033' or key is chr(27):
    cleanup()
  elif key is 'q':
    cleanup()
  elif key == 'p' or key == ' ':
    vidpause = 1 - vidpause
  else:
    print("unknown key")

  glutPostRedisplay()

def init():

  global texName

  glEnable(GL_DEPTH_TEST)
# glDisable(GL_DEPTH_TEST)

  glClearColor(0.22, 0.28, 0.34, 0.5)

  glShadeModel(GL_FLAT)

  # obtain texture id from OpenGL
  texName = glGenTextures(1)
# print "got texName = ", texName

  # blending function
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
  glEnable(GL_BLEND)

# glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

def texbind(frame):

  global texName

  # convert OpenCV cv2 frame to OpenGL texture format
# gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
  frame = cv2.flip(frame,0)
  frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
  img = Image.fromarray(frame)

  # img should be compatible with PIL Image
  (imw, imh) = img.size

  # which texture we are manipulating
  glBindTexture(GL_TEXTURE_2D,texName)

  # set up texture parameters
# glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP)
# glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP)
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT)
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT)
# glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST)
# glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST)
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR)

  # transfer image pixels to texture memory
  # target, level, internalFormat, img.w, img.h, border, format, type, pixels
  glTexImage2D(GL_TEXTURE_2D,
               0,
               GL_RGBA,
               imw,imh,
               0,
               GL_RGBA,
               GL_UNSIGNED_BYTE,
               img.convert("RGBA").tobytes())

  # GL_DECAL or GL_REPLACE will mask lighting, use GL_MODULATE instead
# glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
# glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE)
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)

def display():

  global out
  global output

  global findx

  global fk
  global pk
# global points
  global vtd
  global vts
  global scanpath

  gp = None
  width = glutGet(GLUT_WINDOW_WIDTH)
  height = glutGet(GLUT_WINDOW_HEIGHT)

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()
# glOrtho(0, width, 0, height, 0.0, 1.0)
  glOrtho(0, width, 0, height, -1.0, 1.0)

  glMatrixMode(GL_MODELVIEW)
  glLoadIdentity()

  # weird things happen without this
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

  # draw texture map (video frame); depth here is super important!
  glColor4f(1.0, 1.0, 1.0, 1.0)
  glEnable(GL_TEXTURE_2D)
  glBegin(GL_QUADS)

  glTexCoord2f(0.0, 0.0)
  glVertex3f(0.0, 0.0, -0.5)

  glTexCoord2f(1.0, 0.0)
  glVertex3f(width, 0.0, -0.5)

  glTexCoord2f(1.0, 1.0)
  glVertex3f(width, height, -0.5)

  glTexCoord2f(0.0, 1.0)
  glVertex3f(0.0, height, -0.5)
  glEnd()
  glDisable(GL_TEXTURE_2D)

  # advanced pk to current gaze point
  if scanpath is not None:
    setfk(scanpath.fixations)
    setpk(scanpath.gazepoints)

    # get the "current" gaze point (need this for AOI testing below)
    # y-flip
#   gp = (scanpath.gazepoints[pk].x, 1.0 - scanpath.gazepoints[pk].y) \
#        if 0 < pk and pk < len(scanpath.gazepoints) else None
    gp = (scanpath.fixations[fk].x, 1.0 - scanpath.fixations[fk].y) \
         if 0 < fk and fk < len(scanpath.fixations)-1 else None

    # how many seconds to look behind (don't look ahead, doesn't make sense)
    gazewin = 0.300

    # render fixations
    renderFixations(width, height, gazewin, scanpath.fixations)

    # render gazepoints
    renderPoints2D(width, height, gazewin, scanpath.gazepoints)

  # text color
  glColor4f(1.0, 1.0, 1.0, 0.9)

  # text (bitmap)
# font = GLUT_BITMAP_9_BY_15
  font = GLUT_BITMAP_HELVETICA_18
  glRasterPos3f(10.0,10.0,0.9)
  hmsstr = str(vtd).replace('.',',')
# hmsstr = str(vtd)
  infostr = "%s [%d]" % (hmsstr,findx)
  for ch in infostr:
    glutBitmapCharacter(font, ctypes.c_int(ord(ch)))

# font = GLUT_STROKE_ROMAN
# glRasterPos3f(10.0,10.0,0.0)
# for ch in "Hello World":
#   glutStrokeCharacter(font, ctypes.c_int(ord(ch)))

  glFlush()
  glutSwapBuffers()

  # write out the frame, here we would need to screen grab
  if output:
    glReadBuffer(GL_FRONT)
    buf = glReadPixels(0,0,width,height,GL_RGBA,GL_UNSIGNED_BYTE)
    img = Image.frombytes(mode="RGBA",size=(width,height),data=buf)
#   img = img.transpose(Image.FLIP_TOP_BOTTOM)
    # convert OpenGL texture format to OpenCV cv2 frame
#   gray = cv2.cvtColor(np.asarray(img),cv2.COLOR_BGR2GRAY)
    frame = cv2.cvtColor(np.asarray(img),cv2.COLOR_RGB2BGR)
    frame = cv2.flip(frame,0)

    out.write(frame)

def main(argv):

  global cap
  global frame
  global findx
  global size
  global texName
  global out
  global output
  global fk
  global pk
# global points
  global vtd
  global vts
  global vidpause
  global bw_filter_x, bw_filter_y, sg_filter_x, sg_filter_y
  global scanpath
  global herz
  global tdilation
  global fps

  bw_filter_x = BW(2, 60, 8.15)
  bw_filter_y = BW(2, 60, 8.15)
  # dfwidth, dfdegree, dfo
  sg_filter_x = SG(3, 3, 1)
  sg_filter_y = SG(3, 3, 1)

  cap = None
  texName = None
  frame = None
  findx = 0
  output = False
# points = []
  fk = 0
  pk = 0
  vtd = dt.timedelta()
  vts = 0.0
  vidpause = False

  vfile = None
  file = None
  afile = None

  scanpath = None

  width = 640
  height = 360
# herz = 50
  herz = 200
  outdir = './data/'
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
  tdilation = 2.0

  # get args
  try:
    opts, args = getopt.getopt(argv[1:], '', \
                 ['vfile=', 'file=', 'output'])
  except getopt.GetoptError as err:
    print("getopt error: ", err)
    usage(argv)
    cleanup()

  # process args
  for opt,arg in opts:
    if opt == '--output':
      output = True
    elif opt == '--vfile':
      vfile = arg
    elif opt == '--file':
      file = arg
    else:
      sys.argv[1:]

  if output:
    print("Outputting to 'output.mov'")

# print "Processing raw gaze file: ", file
  if file is not None and os.path.isfile(file):

    scanpath = Scanpath()
    scanpath.parseFile(file,width,height,herz)

#   herz = 1.0/scanpath.getsamplingrate()
#   print "Observed samping rate: ", herz
    print("Observed duration: ", scanpath.gettotalduration())
    print("Observed number of gaze points: ", scanpath.getnumgpoints())
    oherz = float(scanpath.getnumgpoints()) / float(scanpath.gettotalduration())
    print("Observed samping rate: ", oherz)

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
#   scanpath.gridify("%s/%s-aois%s" % (outdir,filename,".csv"),\
#                           subj,cond,width,height,xtiles,ytiles)

    scanpath.dumpDAT("%s/%s%s" % (outdir,filename,".dat"),width,height)

    # old bit of code back when we used 'points' to point to either list of
    # gazepoints or fixations
#   points = scanpath.gazepoints

  if vfile is not None and os.path.isfile(vfile):
    # grab video from file
    cap = cv2.VideoCapture(vfile)
  else:
    # gram video from available camera (e.g., laptop cam)
    cap = cv2.VideoCapture(0)

  size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), \
          int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

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
#   cleanup()
  else:
    tdilation = fps / (float(scanpath.gettotalduration()) / (frame_rate * frames))
    print("Observed samping rate: ", tdilation)
    print("desired frame rate and total time: ", \
        (fps/tdilation)*frame_rate, (fps/tdilation)*frame_rate*frames)

  # resize video by (.5,.5)
  size = (size[0]/2, size[1]/2)
  if size[0] != width or size[1] != height:
    print("WARNING: frame and scanpath size mismatch!")
    size = (width, height)
  print("size, fps: ", size, fps)
  cap.set(size[0],size[1])

 # for output
  if output:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # MPEG not supported?
 #  fps = 25.0
    fps = 30.0
    out = cv2.VideoWriter('output.mov',fourcc,fps,size,True)
 #  out = cv2.VideoWriter('output.mov',fourcc,tdilation,size,True)

  # init glut
  glutInit(sys.argv)
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA | GLUT_DEPTH)
  glutInitWindowPosition(500,500)
  window = glutCreateWindow('Python GL Video -- Esc to exit')
  glutSetWindow(window)
  #glutFullScreen()
  glutReshapeWindow(size[0],size[1])

  # set up callbacks
  glutReshapeFunc(reshape)
  glutDisplayFunc(display)
  glutIdleFunc(idle)
  glutKeyboardFunc(keyboard)
# glutMotionFunc(motion)
# glutMouseFunc(mouse)

  init()
  glutMainLoop()

def cleanup():

  # when everything is done, release the capture
  if cap is not None:
    cap.release()

  if output:
    out.release()

  #if using cv2 to play video
# cv2.destroyAllWindows()

  sys.exit()

def idle():

  global cap
  global frame
  global findx
  global size
  global output
  global out
  global vtd
  global vts
  global tdilation
  global fps
  global vidpause

  if cap.isOpened() and not vidpause:

    # capture frame-by-frame
    ret, frame = cap.read()

    if ret is True:

#     print "frame %5d" % (cap.get(cv2.CAP_PROP_POS_FRAMES))
      findx = cap.get(cv2.CAP_PROP_POS_FRAMES)

      # resize frame read in
      frame = cv2.resize(frame, size, 0, 0, cv2.INTER_CUBIC)

      # get frame time stamp (in milliseconds)
      ms = cap.get(cv2.CAP_PROP_POS_MSEC) * (fps/tdilation)
      # rescale into seconds
      vts = ms/1000.0
      # represent ms timestamp as datetime.timedelta (don't += here!)
      vtd = dt.timedelta(milliseconds=ms)
#     print "********************** TIMESTAMP: ", ms

      # our operations on the frame come here
      gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

      ############ chessboard

      # find the chess board corners
#     ret, corners = cv2.findChessboardCorners(gray, (10,7), None)
#     ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
#     ret, corners = cv2.findChessboardCorners(gray, (6,9), \
#     ret, corners = cv2.findChessboardCorners(gray, (9,6), \
#                    cv2.CALIB_CB_ADAPTIVE_THRESH + \
#                    cv2.CALIB_CB_NORMALIZE_IMAGE + \
#                    cv2.CALIB_CB_FILTER_QUADS)
      ret, corners = cv2.findChessboardCorners(gray, (9,6), \
                     cv2.CALIB_CB_ADAPTIVE_THRESH + \
                     cv2.CALIB_CB_NORMALIZE_IMAGE + \
                     cv2.CALIB_CB_FILTER_QUADS)

      # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
#     objp = np.zeros((6*9,3), np.float32)
#     objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
      objp = np.zeros((6*9,3), np.float32)
      objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

      # arrays to store object points and image points from all the images.
      objpoints = [] # 3d point in real world space
      imgpoints = [] # 2d points in image plane.

      # termination criteria for sub pixel corners refinement
      criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

      # if found, add object points, image points (after refining them)
      if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # draw and display the corners
        gray = cv2.drawChessboardCorners(gray, (9,6), corners2,ret)
        cv2.imshow('frame',gray)
#       cv2.waitKey(500)

        ret, mtx, dist, rvecs, tvecs = \
          cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        print("\n")
        print("ret: ", ret)
        print("mtx:\n", mtx)
        print("dist: ", dist)
        print("rvecs: ", rvecs)
        print("tvecs: ", tvecs)

      ############ chessboard

      # display the resulting frame
#     cv2.imshow('frame',gray)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#       break

      texbind(frame)

      glutPostRedisplay()

#######################
# on Windows, see bottom of this web page:
# http://stackoverflow.com/questions/19709026/how-can-i-list-all-available-windows-locales-in-python-console

##locale.setlocale( locale.LC_ALL, 'en_US.UTF-8' )
##locale.setlocale( locale.LC_ALL, 'de_DE' )
#if platform.system() == 'Windows':
#  locale.setlocale( locale.LC_ALL, 'German' )
#else:
#  locale.setlocale( locale.LC_ALL, 'de_DE' )

if __name__ == '__main__':
# main(sys.argv[1:])
  main(sys.argv)
