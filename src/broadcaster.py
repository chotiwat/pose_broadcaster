#!/usr/bin/env python

import rospy
import tf
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
    self.tfListener = tf.TransformListener()
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)
  
  def publish(self, data):
    rospy.logdebug('Publish %s', data)
    self.sock.sendto(cPickle.dumps(data, -1), ('<broadcast>', self.port))

  def format_detection(self, detection):
    try:
      p, o = self.tfListener.lookupTransform('/map', detection.pose.header.frame_id, rospy.Time())
    except tf.Exception as e:
      return None
    return {
      'id': detection.id,
      'time': detection.pose.header.stamp,
      'position': dict(zip('xyz', p)),
      'orientation': dict(zip('xyzw', o))
    }

  def detection_callback(self, data):
    if self.tfListener.frameExists('/map'):
      detections = [self.format_detection(detection) for detection in data.detections]
      detections = filter(bool, detections)
      if len(detections):
        self.publish({
          'frame_id': self.next_frame,
          'detections': detections
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