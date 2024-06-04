#!/usr/bin/env python

import numpy as np

from matplotlib import pyplot as plt
import os

from roboticstoolbox.robot.ERobot import ERobot
import numpy as np

class LBR14820(ERobot):
    """
    Class that imports a LBR URDF model
    ``LBR()`` is a class which imports a LBR Kuka iiwa 14 R820
    .. runblock:: pycon
        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.LBR()
        >>> print(robot)
    """

    def __init__(self):

        folder = f"{os.path.dirname(os.path.abspath(__file__))}/data"
        folder = folder.replace("/build","") #workaround for colcon build in docker
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "iiwa_14.xacro", tld=folder
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Kuka",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            # gripper_links=elinks[9]
        )

        # self.qdlim = np.array([
        #     2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0])

        self.qr = np.array([0.0, -0.7854, 0.0, 1.3962, 0.0, 0.6109, 0.0])
        self.qz = np.zeros(7)

        self.vel_limits = self.toradians(np.array([85,85,100,75,130,135,135]))
        self.acc_limits = np.array([1,1,1,1,1,1,1])
        self.jerk_limits = np.array([2,2,2,2,2,2,2])

        self.frequency = 250

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


    def check_vel_limits(self, velocities):
        for i in range(7):
            if np.any(velocities[:,i] > self.vel_limits[i]) or np.any(velocities[:,i] < -self.vel_limits[i]):
                return False
        return True

    def check_joint_limits(self, pos):
        for i in range(7):
            if np.any(pos[:,i] > self.qlim[1][i]) or np.any(pos[:,i] < self.qlim[0][i]):
                return False
        return True

    def check_acc_limits(self, acc):
        for i in range(7):
            if np.any(acc[:,i] > self.acc_limits[i]) or np.any(acc[:,i] < -self.acc_limits[i]):
                return False
        return True

    def check_limits(self, q,v,a):
        """
        Check joint limits
        :param q: joint positions
        :param v: joint velocities
        :param a: joint accelerations
        :return: True if within limits, False otherwise
        """
        return self.check_joint_limits(q) and self.check_vel_limits(v) and self.check_acc_limits(a)

    

