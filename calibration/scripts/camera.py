import pyrealsense2 as rs
import cv2
from utilt import *
import threading
from collections import deque
import time,os
from functools import partial

file_dir = os.path.dirname(__file__)

class Camera(object):
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
        self.config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.pipeline_profile = self.pipeline.start(self.config)
        self.device = self.pipeline_profile.get_device()
        self.mtx = self.getIntrinsics()
        # pdb.set_trace()
        
        self.hole_filling = rs.hole_filling_filter()

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # cam init
        print('cam init ...')
        i = 30
        while i>0:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            i -= 1
        print('cam init done.')

    def get_data(self, hole_filling=False):
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            if hole_filling:
                depth_frame = self.hole_filling.process(depth_frame)
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            break
        return color_image, depth_image

    def inpaint(self, img, missing_value=0):
        '''
        pip opencv-python == 3.4.8.29
        :param image:
        :param roi: [x0,y0,x1,y1]
        :param missing_value:
        :return:
        '''
        # cv2 inpainting doesn't handle the border properly
        # https://stackoverflow.com/questions/25974033/inpainting-depth-map-still-a-black-image-border
        img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        mask = (img == missing_value).astype(np.uint8)

        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        scale = np.abs(img).max()
        if scale < 1e-3:
            pdb.set_trace()
        img = img.astype(np.float32) / scale  # Has to be float32, 64 not supported.
        img = cv2.inpaint(img, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        img = img[1:-1, 1:-1]
        img = img * scale
        return img

    def getXYZRGB(self,color, depth, robot_pose,camee_pose,camIntrinsics,inpaint=True):
        '''

        :param color:
        :param depth:
        :param robot_pose: array 4*4
        :param camee_pose: array 4*4
        :param camIntrinsics: array 3*3
        :param inpaint: bool
        :return: xyzrgb
        '''
        heightIMG, widthIMG, _ = color.shape
        # heightIMG = 720
        # widthIMG = 1280
        depthImg = depth / 1000.
        # depthImg = depth
        if inpaint:
            depthImg = self.inpaint(depthImg)
        robot_pose = np.dot(robot_pose, camee_pose)

        [pixX, pixY] = np.meshgrid(np.arange(widthIMG), np.arange(heightIMG))
        camX = (pixX - camIntrinsics[0][2]) * depthImg / camIntrinsics[0][0]
        camY = (pixY - camIntrinsics[1][2]) * depthImg / camIntrinsics[1][1]
        camZ = depthImg

        camPts = [camX.reshape(camX.shape + (1,)), camY.reshape(camY.shape + (1,)), camZ.reshape(camZ.shape + (1,))]
        camPts = np.concatenate(camPts, 2)
        camPts = camPts.reshape((camPts.shape[0] * camPts.shape[1], camPts.shape[2]))  # shape = (heightIMG*widthIMG, 3)
        worldPts = np.dot(robot_pose[:3, :3], camPts.transpose()) + robot_pose[:3, 3].reshape(3,
                                                                                              1)  # shape = (3, heightIMG*widthIMG)
        rgb = color.reshape((-1, 3)) / 255.
        xyzrgb = np.hstack((worldPts.T, rgb))
        xyzrgb = self.getleft(xyzrgb)
        return xyzrgb

    def getleft(self, obj1):
        index = np.bitwise_and(obj1[:, 0] < 1.2, obj1[:, 0] > 0.2)
        index = np.bitwise_and(obj1[:, 1] < 0.5, index)
        index = np.bitwise_and(obj1[:, 1] > -0.5, index)
        # index = np.bitwise_and(obj1[:, 2] > -0.1, index)
        index = np.bitwise_and(obj1[:, 2] > 0.24, index)
        index = np.bitwise_and(obj1[:, 2] < 0.6, index)
        return obj1[index]

    def getIntrinsics(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()
        mtx = [intrinsics.width,intrinsics.height,intrinsics.ppx,intrinsics.ppy,intrinsics.fx,intrinsics.fy]
        camIntrinsics = np.array([[mtx[4],0,mtx[2]],
                                  [0,mtx[5],mtx[3]],
                                 [0,0,1.]])
        return camIntrinsics

    def __del__(self):
        self.pipeline.stop()


class CameraL515(object):
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth,1024, 768,rs.format.z16,30)
        self.config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.pipeline_profile = self.pipeline.start(self.config)
        self.device = self.pipeline_profile.get_device()
        # advanced_mode = rs.rs400_advanced_mode(self.device)
        self.mtx = self.getIntrinsics()
        # pdb.set_trace()
        # with open(r"config/d435_high_accuracy.json", 'r') as file:
        #     json_text = file.read().strip()
        # advanced_mode.load_json(json_text)

        self.hole_filling = rs.hole_filling_filter()

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        depth_sensor = self.device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # cam init
        print('cam init ...')
        i = 30
        while i>0:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            # pdb.set_trace()
            color_frame.get_profile().as_video_stream_profile().get_intrinsics()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            i -= 1
        print('cam init done.')
        # pdb.set_trace()

    def getIntrinsics(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()
        mtx = [intrinsics.width,intrinsics.height,intrinsics.ppx,intrinsics.ppy,intrinsics.fx,intrinsics.fy]
        camIntrinsics = np.array([[mtx[4],0,mtx[2]],
                                  [0,mtx[5],mtx[3]],
                                 [0,0,1.]])
        return camIntrinsics

    def get_data(self, hole_filling=False):
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            if hole_filling:
                depth_frame = self.hole_filling.process(depth_frame)
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            break
        return color_image, depth_image

    def get_data1(self, hole_filling=False):
        while True:
            frames = self.pipeline.wait_for_frames()
            # aligned_frames = self.align.process(frames)
            # depth_frame = aligned_frames.get_depth_frame()
            # if hole_filling:
            #     depth_frame = self.hole_filling.process(depth_frame)
            # color_frame = aligned_frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            colorizer = rs.colorizer()
            depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            # depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            break
        return color_image, depth_image

    def inpaint(self, img, missing_value=0):
        '''
        pip opencv-python == 3.4.8.29
        :param image:
        :param roi: [x0,y0,x1,y1]
        :param missing_value:
        :return:
        '''
        # cv2 inpainting doesn't handle the border properly
        # https://stackoverflow.com/questions/25974033/inpainting-depth-map-still-a-black-image-border
        img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        mask = (img == missing_value).astype(np.uint8)

        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        scale = np.abs(img).max()
        if scale < 1e-3:
            pdb.set_trace()
        img = img.astype(np.float32) / scale  # Has to be float32, 64 not supported.
        img = cv2.inpaint(img, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        img = img[1:-1, 1:-1]
        img = img * scale
        return img

    def getXYZRGB(self,color, depth, robot_pose,camee_pose,inpaint=False):
        '''

        :param color:
        :param depth:
        :param robot_pose: array 4*4
        :param camee_pose: array 4*4
        :param camIntrinsics: array 3*3
        :param inpaint: bool
        :return: xyzrgb
        '''
        camIntrinsics = self.mtx
        heightIMG, widthIMG, _ = color.shape
        # pdb.set_trace()
        # heightIMG = 720
        # widthIMG = 1280
        depthImg = depth * self.depth_scale
        # depthImg = depth
        if inpaint:
            depthImg = self.inpaint(depthImg)
        robot_pose = np.dot(robot_pose, camee_pose)

        [pixX, pixY] = np.meshgrid(np.arange(widthIMG), np.arange(heightIMG))
        camX = (pixX - camIntrinsics[0][2]) * depthImg / camIntrinsics[0][0]
        camY = (pixY - camIntrinsics[1][2]) * depthImg / camIntrinsics[1][1]
        camZ = depthImg

        camPts = [camX.reshape(camX.shape + (1,)), camY.reshape(camY.shape + (1,)), camZ.reshape(camZ.shape + (1,))]
        camPts = np.concatenate(camPts, 2)
        camPts = camPts.reshape((camPts.shape[0] * camPts.shape[1], camPts.shape[2]))  # shape = (heightIMG*widthIMG, 3)
        worldPts = np.dot(robot_pose[:3, :3], camPts.transpose()) + robot_pose[:3, 3].reshape(3,
                                                                                              1)  # shape = (3, heightIMG*widthIMG)
        rgb = color.reshape((-1, 3)) / 255.
        xyzrgb = np.hstack((worldPts.T, rgb))
        # xyzrgb = self.getleft(xyzrgb)
        return xyzrgb

    def getleft(self, obj1):
        index = np.bitwise_and(obj1[:, 0] < 1.2, obj1[:, 0] > 0.2)
        index = np.bitwise_and(obj1[:, 1] < 0.5, index)
        index = np.bitwise_and(obj1[:, 1] > -0.5, index)
        # index = np.bitwise_and(obj1[:, 2] > -0.1, index)
        index = np.bitwise_and(obj1[:, 2] > 0.24, index)
        index = np.bitwise_and(obj1[:, 2] < 0.6, index)
        return obj1[index]


    def __del__(self):
        self.pipeline.stop()

def vis_pc(pc):
    import open3d as o3d
    pc1 = o3d.geometry.PointCloud()
    pc1.points = o3d.utility.Vector3dVector(pc)
    o3d.visualization.draw_geometries([pc1])

def publish_pointcloud_ros(cam,color_list,depth_list,xyzrgb_list):
    pointcloud_publisher = rospy.Publisher("l515/pointcloud",PointCloud2,queue_size=1)
    handeye = np.load(os.path.join(file_dir,'..','conig','handeye.npy'))
    euler = tf.transformations.euler_from_matrix(handeye,'sxyz')
    # pose = handeye[:3,3].tolist() + euler.tolist() # [0.5,0,0.5,0,np.pi,np.pi/2]
    pose = [0.5,0,0.5,0,np.pi,np.pi/2]
    frame_id = 'pc_base'
    static_transform(new_frame=frame_id,pose=pose,base_frame='panda_link0')
    rospy.loginfo("Publish pointcloud in /l515/poindcloud in %s frame."%frame_id)
    while not rospy.is_shutdown():
        color, depth = cam.get_data(hole_filling=False)
        color_list.append(color)
        depth_list.append(depth)

        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03),cv2.COLORMAP_JET)
        xyzrgb = cam.getXYZRGB(color[:,:,::-1], depth, np.identity(4), np.identity(4))
        xyzrgb_list.append(xyzrgb)
        
        stamp = rospy.Time.now()
        pub_pc = xyzrgb_array_to_pointcloud2(xyzrgb[:, :3], xyzrgb[:, 3:6], stamp=stamp,frame_id=frame_id)
        pointcloud_publisher.publish(pub_pc)


if __name__ == "__main__":
    
    rospy.init_node('rgbd_camera',anonymous=True)
    cam = CameraL515()

    depth_list = deque(maxlen=2)
    color_list = deque(maxlen=2)
    xyzrgb_list = deque(maxlen=2)

    pub_t = threading.Thread(target=partial(publish_pointcloud_ros,cam,color_list,depth_list,xyzrgb_list))
    pub_t.start()
    
    time.sleep(2)
    while len(color_list)>0:
        # color, depth = cam.get_data(hole_filling=False)

        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03),cv2.COLORMAP_JET)
        # xyzrgb = cam.getXYZRGB(color[:,:,::-1], depth, np.identity(4), np.identity(4))
        
        # stamp = rospy.Time.now()
        # pub_pc = xyzrgb_array_to_pointcloud2(xyzrgb[:, :3], xyzrgb[:, 3:6], stamp=stamp,frame_id=frame_id)
        # pointcloud_publisher.publish(pub_pc)
        # vis_pc(xyzrgb[:,:3])
        # pdb.set_trace()

        # import pdb;pdb.set_trace()
        # cv2.imwrite('../opencv-examples/image/maker1.jpg', color)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_list[-1], alpha=0.03),cv2.COLORMAP_JET)
        cv2.namedWindow('depth')
        cv2.imshow('depth', depth_colormap)
        cv2.namedWindow('color')
        cv2.imshow('color', color_list[-1])
        cv2.waitKey(1)
    # pub_t.stop()