#!/usr/bin/env python

import rospy
import socket
import cPickle
from apriltags_ros.msg import AprilTagDetectionArray

class Broadcaster(object):
  """docstring for Broadcaster"""
  def __init__(self, port = 5555):
    super(Broadcaster, self).__init__()
    self.port = port
    self.next_frame = 1;
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)
  
  def publish(self, data):
    rospy.logdebug('Publish', data)
    self.sock.sendto(cPickle.dumps(data, -1), ('<broadcast>', self.port))

  def format_detection(self, detection):
    p = detection.pose.pose.position
    o = detection.pose.pose.orientation
    return {
      'id': detection.id,
      'time': detection.pose.header.stamp,
      'position': { 'x': p.x, 'y': p.y, 'z': p.z },
      'orientation': { 'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}
    }

  def detection_callback(self, data):
    if len(data.detections):
      self.publish({
        'frame_id': self.next_frame,
        'detections': [self.format_detection(detection) for detection in data.detections]
      })
      self.next_frame += 1

def main():
  rospy.init_node('broadcaster')
  port = rospy.get_param('~port')
  broadcaster = Broadcaster(port)
  rospy.loginfo('Broadcasting to port %d...', port)
  rospy.spin()

if __name__ == '__main__':
  main()