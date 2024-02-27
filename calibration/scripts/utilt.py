import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import tf2_ros,tf
import geometry_msgs.msg
import cv2

def vis_rgb_camera(cam,ret,colors,depths):
    while not ret[-1]:
        color, depth = cam.get_data(hole_filling=False)
        colors.append(color)
        depths.append(depth)
        cv2.imshow('color',color)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


def xyzrgb_array_to_pointcloud2(points, colors, stamp=None, frame_id=None, seq=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    assert(points.shape == colors.shape)

    buf = []

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if seq:
        msg.header.seq = seq
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        N = len(points)
        xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
        msg.height = 1
        msg.width = N

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * N
    msg.is_dense = True
    msg.data = xyzrgb.tostring()

    return msg


def static_transform(new_frame,pose,base_frame='world'):
    '''
    pose : [x,y,z,r,p,y]
    '''
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = base_frame
    static_transformStamped.child_frame_id = new_frame

    static_transformStamped.transform.translation.x = float(pose[0])
    static_transformStamped.transform.translation.y = float(pose[1])
    static_transformStamped.transform.translation.z = float(pose[2])

    quat = tf.transformations.quaternion_from_euler(
                float(pose[3]),float(pose[4]),float(pose[5]))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)