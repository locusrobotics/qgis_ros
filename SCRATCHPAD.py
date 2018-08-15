

# Try to init ROS node or fail out immediately to avoid blocking the UI.
from socket import error as socket_error
import rosgraph
import rospy

try:
    rosgraph.Master('/rostopic').getPid()
except socket_error:
    raise rospy.ROSInitException('Cannot load QGIS ROS. ROS Master was not found.')
else:
    rospy.init_node('qgis_ros_toolbox')
