from camera import CameraL515,Camera
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import os
import os.path as osp

class calibration():
    def __init__(self,mtx,pattern_size=(9,7),square_size=20,handeye='EIH'):
        '''

        :param image_list:  image array, num*720*1280*3
        :param pose_list: pose array, num*4*4
        :param pattern_size: calibration pattern size
        :param square_size: calibration pattern square size, 15mm
        :param handeye:
        '''
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.handeye = handeye
        self.pose_list = []
        self.mtx = mtx
        self.init_calib()

    def init_calib(self):
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = self.square_size * np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        for i in range(self.pattern_size[0] * self.pattern_size[1]):
            x, y = self.objp[i, 0], self.objp[i, 1]
            self.objp[i, 0], self.objp[i, 1] = y, x
        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def detectFeature(self,color,show=True):
        img = color
        self.gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(img, self.pattern_size, None,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH)  # + cv2.CALIB_CB_NORMALIZE_IMAGE+ cv2.CALIB_CB_FAST_CHECK)
        if ret == True:
            # corners2 = corners
            if (cv2.__version__).split('.')[0] == '2':
                # pdb.set_trace()
                cv2.cornerSubPix(self.gray, corners, (5, 5), (-1, -1), self.criteria)
                corners2 = corners
            else:
                corners2 = cv2.cornerSubPix(self.gray, corners, (5, 5), (-1, -1), self.criteria)
            self.imgpoints.append(corners2)
            self.objpoints.append(self.objp)
            if show:
                fig, ax = plt.subplots(figsize=(20, 20))
                ax.imshow(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                plt.title('img with feature point')
                for i in range(self.pattern_size[0] * self.pattern_size[1] - 3):
                    ax.plot(corners2[i, 0, 0], corners2[i, 0, 1], 'r+')
                plt.show()

    def rodrigues_trans2tr(self,rvec, tvec):
        r, _ = cv2.Rodrigues(rvec)
        tvec.shape = (3,)
        T = np.identity(4)
        T[0:3, 3] = tvec
        T[0:3, 0:3] = r
        return T


    def cal(self):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], self.mtx, None)
        # Hm2w = []  # robot end-effector pose
        # pdb.set_trace()
        Hg2c = []
        pose_list = np.array(self.pose_list)
        for i in range(len(rvecs)):
            tt = self.rodrigues_trans2tr(rvecs[i], tvecs[i] / 1000.)
            Hg2c.append(tt)
        Hg2c = np.array(Hg2c)
        rot, pos = cv2.calibrateHandEye(pose_list[:, :3, :3], pose_list[:, :3, 3], Hg2c[:, :3, :3], Hg2c[:, :3, 3])
        camT = np.identity(4)
        camT[:3, :3] = rot
        camT[:3, 3] = pos[:,0]
        return camT


def callback_eepose(state_msg):
    global franka_eepose_list
    O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
    franka_eepose_list.append(O_T_EE)
    

def detect(calib,colors):
    print('---detect feature---')
    calib.detectFeature(colors[-1])
    print('---detect return---')

def record(calib,franka_eepose_list):
    print('---record position---')
    calib.pose_list.append(franka_eepose_list[-1])
    pos_num = len(calib.pose_list)
    img_num = len(calib.imgpoints)
    obj_num = len(calib.objpoints)
    print("pos num:%d, img num:%d, obj num:%d"%(pos_num,img_num,obj_num))
    print('---record return---')

def remove(calib):
    print('---remove feature---')
    calib.imgpoints.pop(-1)
    calib.objpoints.pop(-1)
    pos_num = len(calib.pose_list)
    img_num = len(calib.imgpoints)
    obj_num = len(calib.objpoints)
    print("pos num:%d, img num:%d, obj num:%d" % (pos_num, img_num, obj_num))
    print('---remove return---')


def delete(calib):
    print('---delete position---')
    if len(calib.imgpoints)>0:
        if len(calib.pose_list) != len(calib.imgpoints) or len(calib.pose_list) != len(calib.objpoints):
            print("len(pose_list) != len(imgpoints), please try again")
        calib.pose_list.pop(-1)
        calib.imgpoints.pop(-1)
        calib.objpoints.pop(-1)
        pos_num = len(calib.pose_list)
        img_num = len(calib.imgpoints)
        obj_num = len(calib.objpoints)
        print("pos num:%d, img num:%d, obj num:%d" % (pos_num, img_num, obj_num))
    else:
        print('None postion record')
    print('---delete return---')

def done(calib,result):
    print('---cal hand-eye calibration---')
    pos_num = len(calib.pose_list)
    img_num = len(calib.imgpoints)
    obj_num = len(calib.objpoints)
    print("pos num:%d, img num:%d, obj num:%d" % (pos_num, img_num, obj_num))
    camT = calib.cal()
    result.append(camT)
    print(camT)
    print('---done return---')

if __name__ == "__main__":
    import rospy
    from franka_msgs.msg import FrankaState
    from collections import deque
    from utilt import *
    from functools import partial
    import threading
    from tkinter import *

    franka_eepose_list = deque(maxlen=2)
    
    rospy.init_node("calib_node",anonymous=True)
    cam = CameraL515()
    calib = calibration(cam.mtx)
    # calib_image_root = 'image/20210908/calib'
    colors = deque(maxlen=2)
    depths = deque(maxlen=2)
    ret_vis = [False]
    result = []
    vis_t = threading.Thread(target=partial(vis_rgb_camera, cam, ret_vis,colors,depths))
    vis_t.start()

    sub_eepose = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState,callback_eepose,queue_size=1)

    # button for record and check
    root = Tk()
    root.title("Check & Record")
    root.geometry("700x75+1000+0")
    B1 = Button(root, text="Detect", command=partial(detect,calib,colors))
    B1.place(x=30, y=20)

    B2 = Button(root, text="Record", command=partial(record,calib,franka_eepose_list))
    B2.place(x=160, y=20)

    B3 = Button(root, text="Remove", command=partial(remove, calib))
    B3.place(x=290, y=20)

    B4 = Button(root, text="Delete", command=partial(delete,calib))
    B4.place(x=420, y=20)

    B5 = Button(root, text="Done", command=partial(done,calib,result))
    B5.place(x=550, y=20)
    root.mainloop()
    ret_vis.append(True)
    import pdb;pdb.set_trace()
    # for i in range(int(num)):
    #     current_pose = franka_eepose_list[-1]
    #     # time.sleep(1)
    #     color, depth = colors[-1],depths[-1]
    #     calib.detectFeature(color)
    #     calib.pose_list.append(current_pose)
    #     import pdb;pdb.set_trace()
    #
    #     # os.makedirs(calib_image_root, exist_ok=True)
    #     # os.makedirs(osp.join(calib_image_root, 'color'), exist_ok=True)
    #     # os.makedirs(osp.join(calib_image_root, 'depth'), exist_ok=True)
    #     # cv2.imwrite(osp.join(calib_image_root, 'depth', 'franka_%0.3d.png' % (i + 1)), depth)
    #     # cv2.imwrite(osp.join(calib_image_root, 'color', 'franka_%0.3d.png' % (i + 1)), color)
    #     # pose_list.append(current_pose)
    #     # cv2.imshow(color)
    #     # cv2.waitKey()
    #
    # camT = calib.cal()
    # import pdb;pdb.set_trace()
    # # np.save(osp.join(calib_image_root,'pose_list.npy'), np.array(pose_list))
    # np.save('config/campose20210909_franka.npy',camT)