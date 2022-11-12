#!/usr/bin/env python
# aoi.py
# This class represents an AOI

# system includes
import sys, math
import numpy as np

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import cv2
import cv2.aruco as aruco

class ArucoAOI:
  def __init__(self,ix,iy,w,h,scalar,ilabel):
    x,y = ix*scalar, iy*scalar
    self.xy = (x,y)
#   self.xy = (x,y-h)
    self.width = w
    self.height = h
    self.label = ilabel

    # specify corners as tl, tr, br, bl (0,0 at upper left)
    self.verts = np.array([ [ (x  )*scalar, (y  )*scalar, 0.0],\
                            [ (x+w)*scalar, (y  )*scalar, 0.0],\
                            [ (x+w)*scalar, (y+h)*scalar, 0.0],\
                            [ (x  )*scalar, (y+h)*scalar, 0.0] ])

    # specify corners as tl, tr, br, bl (0,0 at bottom left)
#   self.verts = np.array([ [ (x  )*scalar, (y-h)*scalar, 0.0],\
#                           [ (x+w)*scalar, (y-h)*scalar, 0.0],\
#                           [ (x+w)*scalar, (y  )*scalar, 0.0],\
#                           [ (x  )*scalar, (y  )*scalar, 0.0] ])

    self.tv = None

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

  def disappear(self):
    self.tv = None

  def dump(self):
    for i in range(len(self.verts)):
      print(self.verts[i])
    print(self.label, ": (x,y), (w,h) = ", \
                      self.xy, ",", self.width, ",", self.height)
		
  def project(self,rvec,tvec,camMat,distCoeffs):
    self.tv, jac = cv2.projectPoints(self.verts,rvec,tvec,camMat,distCoeffs)

  # this function assumes verts are NOT normalized and in image space
  def renderFilled(self,w,h):

    if self.tv is None:
      return

    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
    glBegin(GL_POLYGON)
    for k in range(4):
      # y-flip
      v = np.array([self.tv[k][0][0], h - self.tv[k][0][1]])
      glVertex3f(v[0], v[1], 0.0)
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

  # this function assumes verts are NOT normalized and in image space
  def render(self,w,h):

    if self.tv is None:
      return

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
      v = np.array([self.tv[k][0][0], h - self.tv[k][0][1]])
      glVertex3f(v[0], v[1], 0.0)
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)


  def draw(self,frame):

    if self.tv is None:
      return

    tl = (int(self.tv[0][0][0]),int(self.tv[0][0][1]))
    tr = (int(self.tv[1][0][0]),int(self.tv[1][0][1]))
    br = (int(self.tv[2][0][0]),int(self.tv[2][0][1]))
    bl = (int(self.tv[3][0][0]),int(self.tv[3][0][1]))
    # don't use cv2.rectangle since that draws upright rectangle
    frame = cv2.line(frame,tl,tr,(255,255,255))
    frame = cv2.line(frame,tr,br,(255,255,255))
    frame = cv2.line(frame,br,bl,(255,255,255))
    frame = cv2.line(frame,bl,tl,(255,255,255))

    return frame

  def drawFilled(self,frame):

    if self.tv is None:
      return

    tl = (int(self.tv[0][0][0]),int(self.tv[0][0][1]))
    tr = (int(self.tv[1][0][0]),int(self.tv[1][0][1]))
    br = (int(self.tv[2][0][0]),int(self.tv[2][0][1]))
    bl = (int(self.tv[3][0][0]),int(self.tv[3][0][1]))
    # don't use cv2.rectangle since that draws upright rectangle
    poly = np.array([ [tl[0],tl[1]], \
                      [tr[0],tr[1]], \
                      [br[0],br[1]], \
                      [bl[0],bl[1]] ])
    frame = cv2.fillConvexPoly(frame,poly,(255,255,255))

    return frame

  def inside(self,x,y):

#   print("inside: (%f, %f)" % (x,y))
#   print("tv: ", self.tv)

    if self.tv is None:
      return False

    theta = 0
    epsilon = 0.0001

    n = len(self.tv)

    for i in range(n):
      # an np.array([x,y]) is a 2D vector
      v1 = np.array([self.tv[i][0][0] - x,self.tv[i][0][1] - y])

      # don't forget the angle between the last and first vertex
      if (i+1) < n:
        v2 = np.array([self.tv[i+1][0][0] - x,self.tv[i+1][0][1] - y])
      else:
        v2 = np.array([self.tv[0][0][0] - x,self.tv[0][0][1] - y])

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
