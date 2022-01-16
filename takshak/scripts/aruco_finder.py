#!/usr/bin/env python

# ROS node to subscribe to the raw image, find the aruco markers and get the corresponding
# BGR value. Then publish to a topic /DictPub 
import roslib
roslib.load_manifest('takshak')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from takshak.msg import IntColor
import numpy as np

class image_converter:

  def __init__(self):

    rospy.init_node('aruco_finder', anonymous=True)
    self.dict_pub = rospy.Publisher("/ArucoDict",IntColor,queue_size=10)
    self.Dict = IntColor()

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
  
  def callback(self,data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict,parameters=arucoParams)
    
    markerList = []
    colourList = []

    if len(corners) > 0:
        ids = ids.flatten()

        for(markerCorner, markerID) in zip(corners, ids):
          corners = markerCorner.reshape((4, 2))
          (topLeft, topRight, bottomRight, bottomLeft) = corners
          
          dist_up = int(topRight[1]) - int(bottomRight[1]) 
          topRight = (int(topRight[0]), int(topRight[1]))
          bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
          bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
          topLeft = (int(topLeft[0]), int(topLeft[1]))
          cX = int((topLeft[0] + bottomRight[0]) / 2.0)
          cY = int((topLeft[1] + bottomRight[1]) / 2.0)
          # dist_up = topRight[1] - bottomRight[1] 
          dist_up = int(dist_up * 1.5)
          print(dist_up)

        
          markerList.append(markerID)
          [b,g,r] = (cv_image[cY+dist_up,cX])
          colourList.append([b,g,r])

    #       cv2.putText(cv_image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
    #                           0.5, (0, 255, 0), 1)
    # cv2.imshow("frame",cv_image)
    # cv2.waitKey(3)
    

    try:
        flat_colorList = list(np.concatenate(colourList).flat)
        self.Dict.num=markerList
        self.Dict.rgb=flat_colorList

        try:
          if(len(self.Dict.num) == 5):
            self.dict_pub.publish(self.Dict)
            print("Published !!!")
        except CvBridgeError as e:
            print(e)
    except ValueError:
        pass

def main():
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()