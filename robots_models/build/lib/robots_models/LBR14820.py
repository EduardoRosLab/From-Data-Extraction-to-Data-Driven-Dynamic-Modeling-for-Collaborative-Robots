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

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

