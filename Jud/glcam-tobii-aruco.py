#!/usr/bin/env python

import platform, sys, os, math
import locale
import getopt
import numpy as np
import collections
import cv2
import cv2.aruco as aruco
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
from arucoaoi import ArucoAOI
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

global ak
global fk
global pk
#global points
global bw_filter_x, bw_filter_y, sg_filter_x, sg_filter_y
global scanpath

global fx, fy
global cx, cy
global cameraMatrix
global k1, k2, k3
global p1, p2
global distCoeffs
global markerLength
global arAOI
global aruco_dict_org
global aruco_parameters
global origin
global x_axis
global y_axis
global z_axis

global rvec_u
global rvec_k

def usage(argv):
  print("Usage: ", argv[0], "\n" \
        "  --vfile=?\n" \
        "  --file=?\n" \
        "  --output\n" \
        "    vfile: video input file\n" \
        "    file: (raw) gaze input file\n" \
        "    output: if present output video\n")

# from: https://stackoverflow.com/questions/50764623/object-is-wrong-displaced-in-ar-aruco-opengl
def compositeArray(rvec, tvec):
  v = np.c_[rvec, tvec.T]
  #print(v)
  v_ = np.r_[v, np.array([[0,0,0,1]])]
  return v_

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
  gluDisk(quadric,radius-3,radius,slices,1)
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
#   print("fixating: YES")
    return True

def setfk(points):

  global fk
  global vts

  # debug
# print("setfk: vts = ", vts)
# print("setfk: fk = ", fk)
# print("setfk: points[0].t = ", points[0].t)

  if len(points) > 0:

    # use time relative to beginning (synching scanpath to video stream)
#   while fk < len(points)-1 and (points[fk].t - points[0].t) < vts:
#   while fk < len(points)-1 and points[fk].t < vts:
    while fk < len(points)-1 and points[fk].t + points[fk].duration < vts:
#   while fk < len(points)-1 and mt < vts:
#     print("vts, points.t, fk: ", vts, (points[fk].t - points[0].t), fk)
      fk += 1

def setpk(points):

  global pk
  global vts

  if len(points) > 0:

    # use time relative to beginning (synching scanpath to video stream)
#   while pk < len(points)-1 and (points[pk].t - points[0].t) < vts:
    while pk < len(points)-1 and points[pk].t < vts:
#     print("vts, points.t, pk: ", vts, (points[pk].t - points[0].t), pk)
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
#     print("!!!!!!!!! got point", points[dk].x, " , ", points[dk].y)
      dt = points[dk].t - points[dk-1].t if dk-1 >= 0 else points[dk].t
      dur += dt

      t = max(1.0 - dur/gazewin,0.5)

      glColor4f(241./256., 163./256., 64./256., t)

#     glBegin(GL_POINTS)
#     glVertex2f(points[dk].x*width, points[dk].y*height)
#     glEnd()

      x, y = points[dk].x, points[dk].y
      dur = points[dk].getPercentDuration()

      glPushMatrix()
      # y-flip
      glTranslatef(x*width, (1. - y)*height, 0.0)
#     glDrawCircle(30,36)
      glDrawCircle(dur * d/6.0,18)
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
#       print("!!!!!!!!! got point", points[dk].x, " , ", points[dk].y)
        dt = points[dk].t - points[dk-1].t if dk-1 >= 0 else points[dk].t
        dur += dt

        t = max(1.0 - dur/gazewin,0.5)

#       print("t, dt, dur, gazewin: ", t, dt, dur, gazewin)
        glColor4f(0.7, 0.7, 0.7, t)

#       glBegin(GL_POINTS)
#       glVertex2f(points[dk].x*width, points[dk].y*height)
#       glEnd()

        x, y = points[dk].x, points[dk].y
        xp, yp = points[dk-1].x, points[dk-1].y

        if x > 0.1 and y > 0.1 and xp > 0.1 and yp > 0.1:

          glPushMatrix()
          # y-flip
          glTranslatef(x*width, (1. - y)*height, 0.0)
          glDrawDisc(3,36)
          glPopMatrix()

          glBegin(GL_LINES)
          # y-flip
          glVertex2f(x*width, (1. - y)*height)
          glVertex2f(xp*width, (1. - yp)*height)
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

def keyboard(bkey, x, y):

  global vidpause

  key = bkey.decode("utf-8")

  if key == '\033' or key == chr(27):
    cleanup()
  elif key == 'q':
    cleanup()
  elif key == 'p' or key == ' ':
    vidpause = 1 - vidpause
  else:
    print("unknown key: %c" % (key))

  glutPostRedisplay()

def init():

  global texName

  glEnable(GL_DEPTH_TEST)
# glDisable(GL_DEPTH_TEST)

  glClearColor(0.22, 0.28, 0.34, 0.5)

  glShadeModel(GL_FLAT)

  # obtain texture id from OpenGL
  texName = glGenTextures(1)
# print("got texName = ", texName)

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

  global ak
  global fk
  global pk
# global points
  global vtd
  global vts
  global scanpath

  global fx, fy
  global cx, cy
  global cameraMatrix
  global k1, k2, k3
  global p1, p2
  global distCoeffs
  global markerLength
  global arAOI
  global aruco_dict_org
  global aruco_parameters
  global origin
  global x_axis
  global y_axis
  global z_axis

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
#   gp = (scanpath.fixations[fk].x, 1.0 - scanpath.fixations[fk].y) \
#        if 0 < fk and fk < len(scanpath.fixations)-1 else None
    gp = (scanpath.fixations[fk].x, \
   1.0 - (scanpath.fixations[fk].y)) \
         if 0 < fk and fk < len(scanpath.fixations)-1 else None

    # how many seconds to look behind (don't look ahead, doesn't make sense)
    gazewin = 0.300

    # render fixations
    renderFixations(width, height, gazewin, scanpath.fixations)

    # render gazepoints
    renderPoints2D(width, height, gazewin, scanpath.gazepoints)

  # draw aruco axes
  """
  if origin is not None:
    glLineWidth(4)
    glColor4f(1.0, 0.0, 0.0, 0.8)
    glBegin(GL_LINES)
    # y-flip
    glVertex2f(origin[0],height - origin[1])
    glVertex2f(x_axis[0],height - x_axis[1])
    glEnd()
    glColor4f(0.0, 1.0, 0.0, 0.8)
    glBegin(GL_LINES)
    # y-flip
    glVertex2f(origin[0],height - origin[1])
    glVertex2f(y_axis[0],height - y_axis[1])
    glEnd()
    glColor4f(0.0, 0.0, 1.0, 0.8)
    print("z_axis: (%f, %f)\n" % (z_axis[0] - origin[0], \
                                 (height - z_axis[1]) - (height - origin[1])))
    glBegin(GL_LINES)
    # y-flip
    glVertex2f(origin[0],height - origin[1])
    glVertex2f(z_axis[0],height - z_axis[1])
    glEnd()
    glLineWidth(1)
  """

  # draw aruco markers
  glColor4f(1.0, 1.0, 1.0, 0.4)
  glDisable(GL_DEPTH_TEST)

  for i in range(8):
    if arAOI[i] is not None:
      # need to scale gp coords and do y-flip for inside test
      if gp is not None and arAOI[i].inside(gp[0]*width,height - gp[1]*height):
        arAOI[i].renderFilled(width,height)
      else:
        arAOI[i].render(width,height)

  glEnable(GL_DEPTH_TEST)

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

def parseafile(afile):

  print("Processing: ", afile)

  linelist = f.read().splitlines()
  header = linelist[0].split(',')
  linelist = linelist[1:]

  for idx, label in enumerate(header):
    if "timestamp" in label.strip():
      TIMESTAMP = idx
    if "frame" in label.strip():
      FRAME = idx
    if "label" in label.strip():
      LABEL = idx
    if "duration" in label.strip():
      DURATION = idx

  for line in linelist:
    elements = line.split(',')
    entry = np.asarray(elements)

    # data already normalized, no need to divide by screen size
    t = entry[TIMESTAMP]
    r = entry[FRAME]
    a = entry[LABEL]
    d = entry[DURATION]

    anns.append(ANN(int(r),float(t),a,float(d)))

  return anns

def main(argv):

  global cap
  global frame
  global findx
  global size
  global texName
  global out
  global output
  global ak
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

  global fx, fy
  global cx, cy
  global cameraMatrix
  global k1, k2, k3
  global p1, p2
  global distCoeffs
  global markerLength
  global arAOI
  global aruco_dict_org
  global aruco_parameters
  global origin
  global x_axis
  global y_axis
  global z_axis

  global rvec_u
  global rvec_k

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
  markerLength = 1.0
# markerLength = 10.0
# markerLength = 0.01
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

  arAOI = []

  # xoffset, yoffset, w, h, units are relative to markerLength, hence
  # w=10, h=10 means 10 x w, 10 x h

  # for marker 0
  
  ar = ArucoAOI(0.0, 0.0, 1.0, 1.0, markerLength, "arAOI0")
  arAOI.append(ar)

  # for marker 1
  ar = ArucoAOI(0, 0, 1.0, 1.0, markerLength, "arAOI1")
  arAOI.append(ar)

  # for marker 2
  ar = ArucoAOI(-0, 0, 1.0, 1.0, markerLength, "arAOI2")
  arAOI.append(ar)

  # for marker 3
  ar = ArucoAOI(0, 0, 1.0, 1.0, markerLength, "arAOI3")
  arAOI.append(ar)

  # for marker 4
  ar = ArucoAOI(0, 0, 1.0, 1.0, markerLength, "arAOI4")
  arAOI.append(ar)

  # for marker 5
  ar = ArucoAOI(0, 0, 1.0, 1.0, markerLength, "arAOI5")
  arAOI.append(ar)

  # for marker 6
  ar = ArucoAOI(0, 0, 1.0, 1.0, markerLength, "arAOI6")
  arAOI.append(ar)

  # for marker 7
  ar = ArucoAOI(0, 0, 1.0, 1.0, markerLength, "arAOI7")
  arAOI.append(ar)

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
  ak = 0
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
  height = 480
  herz = 50
  outdir = './data/'
  sfdegree = 2
  sfcutoff = 6.15 # less smooth
  dfo = 1
  dfwidth = 3
  dfdegree = 3
  smooth = False
  screen = 27
  dist = 23.62
  T = 20

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

# print("Processing raw gaze file: ", file)
  if file is not None and os.path.isfile(file):

    scanpath = Scanpath()
    scanpath.parseFile(file,width,height,herz)

#   herz = 1.0/scanpath.getsamplingrate()
#   print("Observed samping rate: ", herz)
    print("Observed duration: ", scanpath.gettotalduration())
    print("Observed number of gaze points: ", scanpath.getnumgpoints())
    herz = float(scanpath.getnumgpoints()) / float(scanpath.gettotalduration())
    print("Observed samping rate: ", herz)

    base = os.path.basename(file)
    print("Processing: ", file, "[", base, "]")

    # split filename from extension
    filename, ext = os.path.splitext(base)

    scanpath.smooth("%s/%s-smth%s" % (outdir,filename,".dat"),\
                    width,height,herz,sfdegree,sfcutoff, smooth)
    scanpath.differentiate("%s/%s-diff%s" % (outdir,filename,".dat"),\
                            width,height,screen,dist,herz,dfwidth,dfdegree,dfo)
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
  size = (size[0]//3, size[1]//3)
  print("size, fps: ", size, fps)
  cap.set(size[0],size[1])

 # for output
  if output:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # MPEG not supported?
 #  fps = 25.0
    # we're slowing down the video to the frame rate of the eye tracker
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

# sys.exit()
  os._exit(0)

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

  global fx, fy
  global cx, cy
  global cameraMatrix
  global k1, k2, k3
  global p1, p2
  global distCoeffs
  global markerLength
  global arAOI
  global aruco_dict_org
  global aruco_parameters
  global origin
  global x_axis
  global y_axis
  global z_axis

  global rvec_u
  global rvec_k

  width = glutGet(GLUT_WINDOW_WIDTH)
  height = glutGet(GLUT_WINDOW_HEIGHT)

  candidateFaces = []

  if cap.isOpened() and not vidpause:

    # capture frame-by-frame
    ret, frame = cap.read()

    if ret is True:

#     print("frame %5d" % (cap.get(cv2.CAP_PROP_POS_FRAMES)))
      findx = cap.get(cv2.CAP_PROP_POS_FRAMES)

      # resize frame read in
      frame = cv2.resize(frame, size, 0, 0, cv2.INTER_CUBIC)

      # get frame time stamp (in milliseconds)
      ms = cap.get(cv2.CAP_PROP_POS_MSEC) * (fps/tdilation)
      # rescale into seconds
      vts = ms/1000.0
      # represent ms timestamp as datetime.timedelta (don't += here!)
      vtd = dt.timedelta(milliseconds=ms)
#     print("********************** TIMESTAMP: ", ms)

      # our operations on the frame come here
      gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

      #################### ARUCO ########################
      origin = None
      x_axis = None
      y_axis = None
      z_axis = None
      # aruco marker detection
      corners_org, ids_org, rejectedImgPoints_org = \
        aruco.detectMarkers(frame, aruco_dict_org, parameters=aruco_parameters)
#     print("CORNERS_ORG:")
#     print(corners_org)

     # note: this will draw false positives as well (annoying)
#     frame = aruco.drawDetectedMarkers(frame,corners_org)
      frame = aruco.drawDetectedMarkers(frame,corners_org,ids_org)

      # pose estimation
      # opencv v3.4.1.2
      rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners_org,\
                                                   markerLength,\
                                                   cameraMatrix,\
                                                   distCoeffs)

      # I think I see the problem: need to process for each marker!
      if rvec is not None and tvec is not None:
        ids_seen_already = []
        for i in range(len(ids_org)):
#         print("ids_org = ", ids_org, "(", len(ids_org), ")")
          # skip id 0, seems like false positive
#         if ids_org[i] == 0:
#           continue
          # in this video we only use ids 0, 1, 2, 3, 4
          if ids_org[i] < 0 or 7 < ids_org[i]:
            continue
          # skip duplicates
          if ids_org[i] in ids_seen_already:
            continue
          ids_seen_already.append(ids_org[i])

          print(" rvec_u: ", rvec_u)
          print("rvec[i]: ", rvec[i][0])
          a, b, c = rvec[i][0][0], rvec[i][0][1], rvec[i][0][2]
          theta = math.sqrt(a*a + b*b + c*c)
          print("rotation angle theta: ", theta)
          # here I was trying to use the last rvec if c > 0.0, thinking that
          # maybe c > 0.0 was indicating the ambiguity (doesn't seem to work)
#         if c > 0.0:
#           rvec[i][0] = rvec_u if rvec_u is not None else rvec[i][0]
#         else:
#           rvec_u = rvec[i][0]
          rmat, jac = cv2.Rodrigues(rvec[i])

          # try the code below or use CORNER_REFINE_SUBPIX
#         # from https://github.com/opencv/opencv/issues/8813
#         rmat, jac = cv2.Rodrigues(rvec[i])
          print("rmat before fix:\n", rmat)
#         print "rmat determinant: ", cv2.determinant(rmat)
          # swap axes; makes y the up axis, z forward
          rmat  = rmat.dot(np.array([ [  1.0,  0.0,  0.0],\
                                      [  0.0,  0.0,  1.0],\
                                      [  0.0, -1.0,  0.0] ]))
          print("rmat after axis swap:\n", rmat)
#         if 0.0 < rmat[1][1] and rmat[1][1] < 1.0:
          if False:
            # if we fall in here,  pose is flipped, negate [1,2]
            print("######### POSE IS FLIPPED ##########")
            # flip the axes, e.g., y axis becomes [-y0, -y1, y2]
            rmat *= np.array([ [  1.0, -1.0,  1.0],\
                               [  1.0, -1.0,  1.0],\
                               [ -1.0,  1.0, -1.0] ])
#           rmat = rmat.dot(np.array([ [  1.0, -1.0,  1.0],\
#                              [  1.0, -1.0,  1.0],\
#                              [ -1.0,  1.0, -1.0] ]))
            # fixup: rotate along the plane spanned by camera's forward (z) axis
            # and vector to marker's position
            forward = np.array([0.0, 0.0, 1.0])
            print("tvec: ", tvec)
            tnorm = tvec / np.linalg.norm(tvec)
            print("tnorm: ", tnorm)
            axis = np.cross(tnorm, forward)
            print("axis: ", axis)
            angle = -2.0*math.acos(tnorm.dot(forward))
            print("angle: ", angle)
            print("angle * axis: ", angle * axis)
            rmat = cv2.Rodrigues(angle * axis)[0].dot(rmat)
            # swap axes back
            rmat  = rmat.dot(np.array([ [  1.0,  0.0,  0.0],\
                                        [  0.0,  0.0, -1.0],\
                                        [  0.0,  1.0,  0.0] ]))
            print("rvec[i]: ", rvec[i])
            print("cv2.Rodrigues(rmat): ", cv2.Rodrigues(rmat))
            print("cv2.Rodrigues(rmat)[0]: ", cv2.Rodrigues(rmat)[0])
            print("cv2.Rodrigues(rmat)[1]: ", cv2.Rodrigues(rmat)[1])
#           rvec[i] = cv2.Rodrigues(rmat)[0].reshape(1,3)
            rvec[i] = cv2.Rodrigues(rmat)[0].T
#         print("tvec = ", tvec[i])
#         rmat, jac = cv2.Rodrigues(rvec[i])
#         zdir = rmat.dot(np.array([ 0.0, 0.0, 1.0]))
#         print("zdir: ", zdir)
          # see:
      # http://cs-courses.mines.edu/csci507/projects/2016/Cairns_VanderMeer.pdf
      # they say that the detected z-axis of the marker flips
      # constraining the change in z-axis to a certain threshold between frames
      #   solves the issue

#         print("cameraMatrix = \n", cameraMatrix)
#         print("distCoeffs = ", distCoeffs)
          print("***** MARKER LENGTH: ", markerLength)
#         frame = aruco.drawAxis(frame,cameraMatrix,distCoeffs,\
#                                rvec[i],tvec[i],markerLength)
          frame = cv2.drawFrameAxes(frame,cameraMatrix,distCoeffs,\
                                 rvec[i],tvec[i],markerLength)

          # draw my own axes
          axis = np.array([ [ 0.0, 0.0, 0.0],\
                            [ markerLength, 0.0, 0.0],\
                            [ 0.0, markerLength, 0.0],\
                            [ 0.0, 0.0, markerLength] ])
          taxis, jac = cv2.projectPoints(axis,\
                                         rvec[i],tvec[i],\
                                         cameraMatrix,distCoeffs)
#         print("taxis\n")
#         print(taxis)
#         print("taxis[0]\n")
#         print(taxis[0])
          # strip off extra [] for OpenCV
          origin = taxis[0][0]
          x_axis = taxis[1][0]
          y_axis = taxis[2][0]
          z_axis = taxis[3][0]
#         print("origin\n", origin)
#         print("x_axis\n", x_axis)
#         print("y_axis\n", y_axis)
#         print("z_axis\n", z_axis)
#         pt1 = (int(origin[0]),int(origin[1]))
#         pt2 = (int(x_axis[0]),int(x_axis[1]))
#         frame = cv2.line(frame,pt1,pt2,(0,0,255))
#         pt1 = (int(origin[0]),int(origin[1]))
#         pt2 = (int(y_axis[0]),int(y_axis[1]))
#         frame = cv2.line(frame,pt1,pt2,(0,255,0))
#         pt1 = (int(origin[0]),int(origin[1]))
#         pt2 = (int(z_axis[0]),int(z_axis[1]))
#         frame = cv2.line(frame,pt1,pt2,(255,0,0))

          if 0 <= ids_org[i] and ids_org[i] <= 7:
            arAOI[int(ids_org[i])].project(rvec[i],tvec[i],\
                      cameraMatrix,distCoeffs)
          else:
            print("UNKNOWN MARKER")

          # get OpenGL rotation matrix from rvec
          # see also:
          # http://answers.opencv.org/question/23089/opencv-opengl-proper-camera-pose-using-solvepnp/
#         rmat, jac = cv2.Rodrigues(rvec[i])
          #print("rmat:\n", rmat)
          # from: https://stackoverflow.com/questions/50764623/object-is-wrong-displaced-in-ar-aruco-opengl
          # fix axis
          tvec[i][0][0] =  tvec[i][0][0]
          tvec[i][0][1] = -tvec[i][0][1]
          tvec[i][0][2] = -tvec[i][0][2]

          rvec[i][0][1] = -rvec[i][0][1]
          rvec[i][0][2] = -rvec[i][0][2]
          m = compositeArray(cv2.Rodrigues(rvec[i])[0],tvec[i][0])
#         print("compositeArray: ", m)

        del ids_seen_already[:]

      else:
        # nothing detected
        for i in range(8):
          arAOI[i].disappear()

      #################### ARUCO ########################

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
