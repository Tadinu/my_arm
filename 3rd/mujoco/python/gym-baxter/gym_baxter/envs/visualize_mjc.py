import mujoco_py
from mujoco_py.mjlib import mjlib
from mujoco_py.mjcore import register_license
from mujoco_py.mjtypes import *
from sys import argv
from mujoco_py.config import init_config, get_key_path
from transformation import quaternion_from_matrix

from ipdb import set_trace

def set_cam_position(viewer, cam_pos):

    for i in range(3):
        viewer.cam.lookat[i] = cam_pos[i]
    viewer.cam.distance = cam_pos[3]
    viewer.cam.elevation = cam_pos[4]
    viewer.cam.azimuth = cam_pos[5]
    viewer.cam.trackbodyid = -1

def print_viewer(viewer):
    x,y,z = viewer.cam.lookat
    d = viewer.cam.distance
    e = viewer.cam.elevation
    a = viewer.cam.azimuth
    print('x:%d,y:%d,z:%d,d:%d,e:%d,a:%d\n'%(x,y,z,d,e,a))

if __name__ == "__main__":

    
    fullpath = 'mjc/baxter/baxter.xml'
    
    if len(argv)>1:
        [fullpath] = argv[1:]

        print( 'file:' + fullpath)
    model = mujoco_py.MjModel(fullpath)
    
   
    viewer = mujoco_py.MjViewer(visible=True)
    viewer.start()
    viewer.set_model(model)

    cam_pos = np.array([1.0, 0.0, 0.7, 0.5, -45, 180])
    # cam_pos = np.array([0.1, 0.0, 0.7, 0.01, -45., 0.])
    set_cam_position(viewer, cam_pos)
    ctrl = model.data.ctrl.copy()
    # ctrl[-1] = 10.0

    model.data.mocap_pos = model.data.site_xpos[3] #np.array([[0.6, 0., 0.]])
    model.data.mocap_quat = quaternion_from_matrix(model.data.site_xmat[3].reshape(3,3))

    # set_trace()

    jt1 = np.array([-0.65597575, -0.77090292, -0.85008688, 1.69193828, 0.70414396, 0.98353719, -0.98110516]).reshape(1,7)
    jt2 = np.array([-0.57907814, -0.55066804, -0.67528671, 1.76927182, 0.94890188, 0.71474327, -1.00205739]).reshape(1,7)
    jt3 = np.array([-0.84808738, -0.3352765 , -0.84940643, 1.42527668, 1.04024461, 0.96507808, -1.19109601]).reshape(1,7)
    
    i = 0

    step_size=1000
    while(True):
        i +=1
        

        if(i==1*step_size):
            # print(model.data.mocap_pos )
            # print(model.data.mocap_quat)
            # ctrl = model.data.ctrl.copy()
            # ctrl[7:14,0] = jt1
            # model.data.ctrl = ctrl
            print("ee position",model.data.site_xpos[0], quaternion_from_matrix(model.data.site_xmat[0].reshape(3,3)))
            print("box position", model.data.site_xpos[3], quaternion_from_matrix(model.data.site_xmat[3].reshape(3,3)))

            # model.data.mocap_pos = model.data.site_xpos[3] #np.array([[0.6, 0., 0.]])
            # model.data.mocap_quat = quaternion_from_matrix(model.data.site_xmat[3])

        # if(i==2*step_size):
        #     print("commanded jt config", jt1)
        #     print("stable jt config", model.data.qpos[10:17].T)
        #     print("qfrc_actuator", model.data.qfrc_actuator[10:17].T)
        #     print("qfrc_applied", model.data.qfrc_applied[10:17].T)
        #     print("qfrc_bias", model.data.qfrc_bias[10:17].T)
        #     print("qfrc_passive", model.data.qfrc_passive[10:17].T)
        #     print("qfrc_unc", model.data.qfrc_unc[10:17].T)
        #     print("qfrc_constraint", model.data.qfrc_constraint[10:17].T)
        #     print("qfrc_inverse", model.data.qfrc_inverse[10:17].T)
            
        #     print("ee position",model.data.site_xpos[0])
        #     print("box position", model.data.site_xpos[3])

            # ctrl = model.data.ctrl.copy()
            # ctrl[7:14,0] = jt2
            # model.data.ctrl = ctrl

        # if(i==3*step_size):
        #     print("commanded jt config", jt2)
        #     print("stable jt config", model.data.qpos[10:17].T)
        #     print("force", model.data.qfrc_actuator.T)
        #     print("qfrc_applied", model.data.qfrc_applied[10:17].T)
        #     ctrl = model.data.ctrl.copy()
        #     ctrl[7:14,0] = jt3
        #     model.data.ctrl = ctrl

        # if(i==6*step_size):
        #     print("commanded jt config", jt3)
        #     print("stable jt config", model.data.qpos[10:17].T)
        #     print("force", model.data.qfrc_actuator[10:17].T)
        #     print("qfrc_applied", model.data.qfrc_applied[10:17].T)
            
        # viewer.loop_once()
        # print(model.data.mocap_pos )
        # print(model.data.mocap_quat) 
        model.step()
        

        
