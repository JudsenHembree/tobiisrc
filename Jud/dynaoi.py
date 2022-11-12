#!/usr/bin/env python
# aoi.py
# This class represents an AOI

# system includes
import sys
import math
import numpy as np
import datetime as dt

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# local includes
from aoi import AOI

class dynAOI:

  def __init__(self,index):

    self.idx = index

    self.tin = None
    self.tout = None
    self.aois = []

  def timein(self,din):

    self.tin = din

  def timeout(self,dout):

    self.tout = dout

  def addAOI(self,aoi):

    self.aois.append(aoi)

  def inside(self,gp):

    # check if we can interpolate
    if len(self.aois) == 2:

      # draw polygon interpolating between our two AOIs
      aoiin = self.aois[0]
      aoiout = self.aois[1]

      # if we have a gazepoint, check to see if it is in either AOI
      if gp is not None and \
        (aoiin.inside(gp[0],gp[1]) or aoiout.inside(gp[0],gp[1])):
        return True
      else:
        return False

  def draw(self,gp,t,w,h):

    # check if we can interpolate
    if len(self.aois) == 2:

      # draw polygon interpolating between our two AOIs
      aoiin = self.aois[0]
      aoiout = self.aois[1]

      # if we have a gazepoint, check to see if it is in either AOI
      if gp is not None and \
        (aoiin.inside(gp[0],gp[1]) or aoiout.inside(gp[0],gp[1])):
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
      else:
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

      glBegin(GL_POLYGON)
      for k in range(4):
        vin = np.array([aoiin.verts[k][0] * w, aoiin.verts[k][1] * h])
        vout = np.array([aoiout.verts[k][0] * w, aoiout.verts[k][1] * h])
        v = (1-t) * vin + (t) * vout
        glVertex2f(v[0], v[1])
      glEnd()
      glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)

  def dump(self):

    print("%d" % (self.idx))
    inhms = str(self.tin).replace('.',',')
    outhms = str(self.tout).replace('.',',')
    print("%s --> %s" % (inhms, outhms))

    for aoi in self.aois:
      print("v %s" % (aoi.tlbrstr()))

    print("")
