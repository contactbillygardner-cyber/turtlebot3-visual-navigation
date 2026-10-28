"""Simulator boundary: pose reads here implement physical sensors or evaluation.
Only camera pixels, encoder angles and LiDAR polar scans leave this boundary.
"""
import xml.etree.ElementTree as ET
import mujoco
import cv2
import numpy as np
from camera_navigation import HERE, MarkerCamera
WIDTH, HEIGHT, FOV = 960, 720, 90.0

def camera_matrix():
    f=HEIGHT/(2*np.tan(np.deg2rad(FOV)/2))
    return np.array([[f,0,(WIDTH-1)/2],[0,f,(HEIGHT-1)/2],[0,0,1]],dtype=float)


def build_room(obstacle_shift=0.0, friction=1.0):
    root = ET.parse(HERE / 'turtlebot3_waffle_pi.xml').getroot()
    root.find('compiler').set('meshdir', str(HERE / 'assets'))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth=str(WIDTH), offheight=str(HEIGHT))
    ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.6 .6 .6')
    ET.SubElement(root, 'statistic', center='1.4 0 .2', extent='3')
    world = root.find('worldbody')
    base = world.find("body[@name='base']")
    ET.SubElement(base, 'camera', name='navigation', pos='.09 0 .30', xyaxes='0 -1 0 -0.173648 0 0.984808', fovy=str(FOV))
    ET.SubElement(base, 'site', name='lidar', pos='-.049 0 .15', size='.005', rgba='0 1 0 1')
    ET.SubElement(world, 'light', pos='0 0 3', directional='true', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .05',
                  rgba='.25 .35 .4 1', friction=f'{friction} .005 .0001')
    assets = root.find('asset')
    ET.SubElement(assets, 'texture', name='marker_texture', type='2d', file=str(HERE/'assets/navigation_marker.png'))
    ET.SubElement(assets, 'material', name='marker_material', texture='marker_texture', texrepeat='1 1',
                  texuniform='false', emission='1', specular='0')
    ET.SubElement(world, 'geom', name='marker', type='plane', pos='3 0 .55', xyaxes='0 -1 0 0 0 1',
                  size='.15 .15 .001', material='marker_material', contype='0', conaffinity='0', group='4')
    boxes = [ ('obstacle_main', (1.25, obstacle_shift, .15), (.22, .25, .15)),
              ('obstacle_side', (2.1, -.95, .15), (.2, .2, .15)),
              ('wall_back', (-.7, 0, .35), (.05, 1.6, .35)),
              ('wall_station', (3.2, 0, .35), (.05, 1.6, .35)),
              ('wall_left', (1.25, 1.6, .35), (2, .05, .35)),
              ('wall_right', (1.25, -1.6, .35), (2, .05, .35)) ]
    for name, pos, size in boxes:
        ET.SubElement(world, 'geom', name=name, type='box', pos=' '.join(map(str,pos)),
                      size=' '.join(map(str,size)), rgba='.65 .35 .15 1', group='5')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


class Lidar:
    angles = np.linspace(-np.pi, np.pi, 181, endpoint=False)
    def __init__(self, model):
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'lidar')
        self.groups = np.array([0, 0, 0, 0, 0, 1], dtype=np.uint8)

    def read(self, model, data, rng, noise=.006):
        # Sensor pose used to cast physical rays, never returned as robot state.
        origin = data.site_xpos[self.site]
        rotation = data.site_xmat[self.site].reshape(3,3)
        ranges = []
        for angle in self.angles:
            direction = rotation @ np.array([np.cos(angle), np.sin(angle), 0.0])
            hit = mujoco.mj_ray(model, data, origin, direction, self.groups, True, -1, None)
            ranges.append(4.0 if hit < 0 else min(4.0, hit))
        return np.clip(np.asarray(ranges) + rng.normal(0, noise, len(ranges)), .03, 4.0)


def evaluation_pose(model, data):
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base')
    rotation = data.xmat[base].reshape(3,3)
    return np.array([*data.xpos[base,:2], np.arctan2(rotation[1,0], rotation[0,0])])


def evaluation_collision(model, data):
    for contact in data.contact:
        if model.geom_group[contact.geom1] == 5 or model.geom_group[contact.geom2] == 5:
            return True
    return False


class CalibratedMarkerCamera(MarkerCamera):
    """Known 22 cm marker pose -> world base pose via PnP and fixed extrinsics."""
    def observe(self, rgb):
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None or 0 not in ids.flatten():
            return None, corners, ids
        index = list(ids.flatten()).index(0)
        ok, rvec, tvec = cv2.solvePnP(self.points, corners[index].reshape(4,2),
                                    camera_matrix(), np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or tvec[2,0]<=0 or not np.isfinite(tvec).all():
            return None, corners, ids
        projected,_ = cv2.projectPoints(self.points,rvec,tvec,camera_matrix(),np.zeros(5))
        error=np.mean(np.linalg.norm(projected.reshape(4,2)-corners[index].reshape(4,2),axis=1))
        if error>3:
            return None,corners,ids
        Rcm,_=cv2.Rodrigues(rvec)
        Rwm=np.array([[0,0,-1],[-1,0,0],[0,1,0]],dtype=float)
        pitch=np.deg2rad(10)
        Rbc=np.array([[0,np.sin(pitch),np.cos(pitch)],[-1,0,0],[0,-np.cos(pitch),np.sin(pitch)]])
        Rwc=Rwm@Rcm.T
        camera_world=np.array([3,0,.55])-Rwc@tvec.flatten()
        Rwb=Rwc@Rbc.T
        heading=np.arctan2(Rwb[1,0],Rwb[0,0])
        base_xy=camera_world[:2]-.09*np.array([np.cos(heading),np.sin(heading)])
        right,down,forward=tvec.flatten()
        horizontal_forward=np.cos(pitch)*forward+np.sin(pitch)*down
        return np.array([np.hypot(right,horizontal_forward), -np.arctan2(right,horizontal_forward),heading]),corners,ids
