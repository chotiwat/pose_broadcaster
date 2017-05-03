# pose_broadcaster
UDP Broadcast Server/Client for Alvar/AprilTag

## Installation
* Install [usb_cam](https://github.com/bosch-ros-pkg/usb_cam).
* Install ROS dependencies: ar_track_alvar apriltags_ros.

## Usage
```bash
roslaunch pose_broadcaster broadcaster.launch
```
### Arguments
* `image_width`   1920
* `image_height`  1280
* `framerate`     15
* `video_device`  /dev/video0
* `use_apriltag`  false
* `tag_family`    36h11 (for AprilTag)
