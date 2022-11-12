#!/usr/bin/env python
# aoi.py
# This class represents an AOI

# system includes
import sys, math
import numpy as np

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

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

class AOI:
  # assume normalized coordinates, (tl), (br) in image coords (0,0) top left
  def __init__(self,list=None,string=None):

    self.xy = (0,0)
    self.width = 0
    self.height = 0
    self.verts = None

    # for decay function (age of the AOI)
    self.age = 0

    if string is not None:
      self.label = string
    else:
      self.label = ""

    if list is not None:
      x1 = list[0]
      y1 = list[1]
      x2 = list[2]
      y2 = list[3]
    else:
      x1, y1, x2, y2 = 0, 0, 0, 0

    self.setCoords(x1,y1,x2,y2)

  def growold(self):
     self.age += 1

  def update(self,other):
    (x1, y1) = other.xy
    x2 = x1 + other.width
    y2 = y1 + other.height
    self.setCoords(x1,y1,x2,y2)

    # if we get new coords, reset age to 0
    self.age = 0

  def setCoords(self,x1,y1,x2,y2):

    # y-flip
#   self.xy = (x1,1-y1)
#   x, y = x1, 1-y1
#   w = x2 - x1
#   h = (1-y2) - y1
    self.xy = (x1,y1)
    x, y = x1, y1
    w = x2 - x1
    h = y2 - y1
    self.width = w
    self.height = h

    # specify corners as tl, tr, br, bl (0,0 at upper left)
    self.verts = np.array([ [ (x  ), (y  ) ],\
                            [ (x+w), (y  ) ],\
                            [ (x+w), (y+h) ],\
                            [ (x  ), (y+h) ] ])

  def getCoords(self):
    (x1, y1) = self.xy
    x2 = x1 + self.width
    y2 = y1 + self.height
    return (x1, y1, x2, y2)

  def movCoords(self,x2,y2):

    (x1, y1) = self.xy
    x, y = x1, y1
    w = x2 - x1
    h = y2 - y1
    self.width = w
    self.height = h

    # specify corners as tl, tr, br, bl (0,0 at upper left)
    self.verts = np.array([ [ (x  ), (y  ) ],\
                            [ (x+w), (y  ) ],\
                            [ (x+w), (y+h) ],\
                            [ (x  ), (y+h) ] ])

  def setAOILabel(self,label):
    self.label = label

  def getAOILabel(self):
    return self.label

  def getXY(self,h=None):
    if h is not None:
      return (self.xy[0],float(h) - self.xy[1])
    else:
      return self.xy

  def getWidth(self):
    return self.width

  def getHeight(self):
    return self.height

  def dump(self):
    for i in range(len(self.verts)):
      print(self.verts[i])
    print(self.label, ": (x,y), (w,h) = ", \
                      self.xy, ",", self.width, ",", self.height)

  def tlbrstr(self):
    tl = (self.verts[0][0], self.verts[0][1])
    br = (self.verts[2][0], self.verts[2][1])
    str = "%3.2f %3.2f %3.2f %3.2f" % (tl[0],tl[1],br[0],br[1])
    return str
		
  # this function assumes verts are NOT normalized and in image space
  def renderFilled(self,w,h):

    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
    glBegin(GL_POLYGON)
    for k in range(4):
      # y-flip
      v = np.array([self.verts[k][0], h - self.verts[k][1]])
      glVertex3f(v[0], v[1], 0.0)
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

  # this function assumes verts are NOT normalized and in image space
  def render(self,w,h):

    # x1,y1
#   (x1, y1, x2, y2) = self.getCoords()
#   glPushMatrix()
#   glTranslate(x2,h-y2,0)
#   glDrawCircle(10,18)
#   glPopMatrix()

    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
    glBegin(GL_POLYGON)
    for k in range(4):
      # y-flip
      v = np.array([self.verts[k][0], h - self.verts[k][1]])
      glVertex3f(v[0], v[1], 0.0)
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)

  # this function assumes verts ARE normalized and in GL space
  def draw(self,w,h):

#   tl = (self.verts[0][0] * w, self.verts[0][1] * h)
#   tr = (self.verts[1][0] * w, self.verts[1][1] * h)
#   br = (self.verts[2][0] * w, self.verts[2][1] * h)
#   bl = (self.verts[3][0] * w, self.verts[3][1] * h)
#
#   glBegin(GL_POLYGON)
#   glVertex2f(tl[0], tl[1])
#   glVertex2f(tr[0], tr[1])
#   glVertex2f(br[0], br[1])
#   glVertex2f(bl[0], bl[1])
#   glEnd()

    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
    glBegin(GL_POLYGON)
    for k in range(4):
      v = np.array([self.verts[k][0] * w, self.verts[k][1] * h])
      glVertex2f(v[0], v[1])
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)

  def drawFilled(self):
#   tl = (self.verts[0][0] * w, self.verts[0][1] * h)
#   tr = (self.verts[1][0] * w, self.verts[1][1] * h)
#   br = (self.verts[2][0] * w, self.verts[2][1] * h)
#   bl = (self.verts[3][0] * w, self.verts[3][1] * h)
#   # don't use cv2.rectangle since that draws upright rectangle
#   poly = np.array([ [tl[0],tl[1]], \
#                     [tr[0],tr[1]], \
#                     [br[0],br[1]], \
#                     [bl[0],bl[1]] ])
#   frame = cv2.fillConvexPoly(frame,poly,(255,255,255))
#
#   return frame

    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
    glBegin(GL_POLYGON)
    for k in range(4):
      v = np.array([self.verts[k][0] * w, self.verts[k][1] * h])
      glVertex2f(v[0], v[1])
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

  def inside(self,x,y):

    theta = 0
    epsilon = 0.0001

    n = len(self.verts)

    for i in range(n):
      v1 = np.array([self.verts[i][0] - x,self.verts[i][1] - y])

      # don't forget the angle between the last and first vertex
      if (i+1) < n:
        v2 = np.array([self.verts[i+1][0] - x,self.verts[i+1][1] - y])
      else:
        v2 = np.array([self.verts[0][0] - x,self.verts[0][1] - y])

      # get dot product
      dot = np.dot(v1,v2)
      v1_modulus = np.sqrt((v1*v1).sum())
      v2_modulus = np.sqrt((v2*v2).sum())
      # cos of angle between x and y
      cos_angle = dot / (v1_modulus * v2_modulus)
      angle = np.arccos(cos_angle)

      # accumulate angle
      theta = theta + angle

    # check to see if we have 360 degrees
#   if abs(theta*180.0/math.pi - 360.0) < epsilon:
    if abs(theta - 2.0*math.pi) < epsilon:
      return True
    else:
      return False
