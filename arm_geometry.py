"""Six-joint forward kinematics from the pinned DM URDF; never controls hardware."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
URDF = ROOT / 'vendor/reBotArm_control_py/urdf/DM/urdf/ReBot_Arm_DM.urdf'
PROFILE = ROOT / 'calibration/arm2_reference.json'


def origin_matrix(element):
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = list(map(float, element.get('xyz', '0 0 0').split()))
    r,p,y = map(float, element.get('rpy', '0 0 0').split())
    result[:3,:3] = (cv2.Rodrigues(np.array([0.,0.,y]))[0] @
                     cv2.Rodrigues(np.array([0.,p,0.]))[0] @
                     cv2.Rodrigues(np.array([r,0.,0.]))[0])
    return result


class Geometry:
    def __init__(self):
        self.profile = json.loads(PROFILE.read_text())
        signs = self.profile.get('signs', [])
        if self.profile.get('physical_arm') != 'second' or len(signs) != 6 or any(s not in (-1,1) for s in signs):
            raise ValueError('Complete second-arm reference and direction checks first')
        self.signs = np.array(signs, dtype=float)
        self.offsets = np.array(self.profile['software_offsets_rad'], dtype=float)
        if self.offsets.shape != (6,) or not np.isfinite(self.offsets).all():
            raise ValueError('Invalid software offsets')
        root = ET.parse(URDF).getroot()
        self.joints = [root.find(f"joint[@name='joint{i}']") for i in range(1,7)]
        self.fingerprint = hashlib.sha256(PROFILE.read_bytes()+URDF.read_bytes()).hexdigest()

    def transform(self, raw):
        raw = np.array(raw, dtype=float)
        if raw.shape != (6,) or not np.isfinite(raw).all():
            raise ValueError('Expected six finite raw joint angles')
        q = raw*self.signs+self.offsets
        T = np.eye(4)
        for joint, angle in zip(self.joints,q):
            T = T @ origin_matrix(joint.find('origin'))
            axis = np.array(list(map(float,joint.find('axis').get('xyz').split())))
            R = np.eye(4)
            R[:3,:3] = cv2.Rodrigues(axis*angle)[0]
            T = T @ R
        return T  # base <- link6 (wrist), NOT fingertip
