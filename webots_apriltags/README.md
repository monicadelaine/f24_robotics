# ROS2 package for webots simulation with apriltags


Launch command
<pre>
ros2 launch webots_apriltags webots_apriltags.launch.py
</pre>

Notes: To get image in Webots, you must subscribe to the camera message.  One way to accomplish this is to echo messages from the topic:

<pre>
ros2 topic echo /TurtleBot3Burger/camera/image_color
</pre>

To add more tags into the environment, reference protos in https://github.com/monicadelaine/webots_apriltag/tree/master/apriltags/protos

Include in wbt file as: 
<pre>EXTERNPROTO "https://raw.githubusercontent.com/monicadelaine/webots\_apriltag/master/apriltags/protos/tag36\_11\_000XX.proto"
</pre>


