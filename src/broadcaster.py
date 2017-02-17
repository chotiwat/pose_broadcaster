#!/usr/bin/env python

import rospy
import tf
import socket
import cPickle
from apriltags_ros.msg import AprilTagDetectionArray

class Broadcaster:
  """docstring for Broadcaster"""
  def __init__(self, frame_prefix, port = 5555):
    self.frame_prefix = frame_prefix
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
      frame = '%s%d' % (self.frame_prefix, detection.id)
      p, o = self.tfListener.lookupTransform('/map', frame, rospy.Time())
      rpy = tf.transformations.euler_from_quaternion(o)
    except tf.Exception as e:
      return None
    return {
      'id': detection.id,
      'time': detection.pose.header.stamp.to_sec(),
      'position': dict(zip('xyz', p)),
      'orientation': dict(zip('rpy', rpy))
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
  frame_prefix = rospy.get_param('~frame_prefix')
  broadcaster = Broadcaster(frame_prefix, port)
  rospy.loginfo('Broadcasting to port %d...', port)
  rospy.spin()

if __name__ == '__main__':
  main()