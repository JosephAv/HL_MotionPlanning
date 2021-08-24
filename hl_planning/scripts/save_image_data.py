#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge     = CvBridge()
    self.image_sub  = rospy.Subscriber("/franka/camera/image_raw", Image,self.callback)
    self.first      = True
    self.idx        = 0
    self.old_img    = np.zeros((0,0))
    self.mag_file   = open("mag_data.csv", "w")
    self.ang_file   = open("ang_data.csv", "w")

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    rospy.logwarn("rows: %s || cols: %s || channels: %s", str(rows), str(cols), str(channels))

    if self.first == True:
      self.first = False
      self.old_img = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    else:
      self.idx      = self.idx + 1
      new_img       = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
      flow          = cv.calcOpticalFlowFarneback(self.old_img, new_img, None, 0.5, 3, 15, 3, 5, 1.2, 0)
      mag, ang      = cv.cartToPolar(flow[...,0], flow[...,1])
      mag           = np.hstack((mag, np.ones((mag.shape[0],1)) * self.idx))
      ang           = np.hstack((ang, np.ones((ang.shape[0],1)) * self.idx))
      self.old_img  = new_img

      np.savetxt(self.mag_file, mag, delimiter = ',')
      np.savetxt(self.ang_file, ang, delimiter = ',')

  def __del__(self):
    self.mag_file.close()
    self.ang_file.close()

def main(args):
  rospy.logwarn("Save Imge node initialization")
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
  f = open("cino.csv","w")
  f.write("Now the file has more content!")
  f.close()

  main(sys.argv)