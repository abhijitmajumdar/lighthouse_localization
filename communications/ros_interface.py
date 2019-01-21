import rospy
import tf
from geometry_msgs.msg import PoseStamped

class ROSInterface:
    def __init__(self):
        rospy.init_node('lighthouse_localization', anonymous=True)
        self.br = tf.TransformBroadcaster()
        self.pose_publisher = rospy.Publisher('pose', PoseStamped, queue_size=1)

    def publish_transform(self, T, src_frame, dest_frame, t=None):
        Q = tf.transformations.quaternion_from_matrix(T)
        self.br.sendTransform(T[:3,3],
                             Q,
                             rospy.Time.now() if t is None else t,
                             dest_frame,
                             src_frame)

    def publish_pose(self,T, frame, t=None):
        Q = tf.transformations.quaternion_from_matrix(T)
        # Q = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
        p = PoseStamped()
        p.header.stamp = rospy.Time.now() if t is None else t
        p.header.frame_id = frame
        p.pose.position.x = T[0][3]
        p.pose.position.y = T[1][3]
        p.pose.position.z = T[2][3]
        p.pose.orientation.x = Q[0]
        p.pose.orientation.y = Q[1]
        p.pose.orientation.z = Q[2]
        p.pose.orientation.w = Q[3]
        self.pose_publisher.publish(p)

    def is_alive(self):
        return not rospy.is_shutdown()
