from typing import *
import math
import photonlibpy
import threading
import dataclasses
import time
import wpimath
import wpimath.geometry
import ncoms
import commands2
import move

class VisionSystem:
    def __init__(self, name: str, sinvert: bool):
        super().__init__()
        self.camera = photonlibpy.PhotonCamera(name)
        self.name = name
        self.sinvert = sinvert

    def __call__(self) -> Optional[Tuple[int, wpimath.geometry.Pose2d, float]]:
        result = self._get()
        ncoms.coproc_tab.putBoolean(self.name + "Connected", self.camera.isConnected())
        if result:
            ncoms.coproc_tab.putNumber(self.name + "Update Status", result[2])
            ncoms.coproc_tab.putBoolean(self.name + "HasTarget", True)
            ncoms.coproc_tab.putNumber(self.name + "Tagid", result[0])
            pose = result[1]
            ncoms.coproc_tab.putNumber(self.name + "posex", pose.X())
            ncoms.coproc_tab.putNumber(self.name + "posey", pose.Y())
            ncoms.coproc_tab.putNumber(self.name + "posea", pose.rotation().degrees())
            return result
        ncoms.coproc_tab.putBoolean(self.name + "HasTarget", False)
        return None

    def _get(self) -> Optional[Tuple[int, wpimath.geometry.Pose2d, float]]:
        print(self.name + "APRIL: Fetching Latest Result")
        recent = self.camera.getLatestResult()
        if not recent.hasTargets(): return
        print(self.name + "APRIL: Acquiring Best Target")
        target = recent.getBestTarget()
        id = target.getFiducialId()
        print(self.name + "APRIL: Calculating the pose")
        raw = target.getBestCameraToTarget()
        x = raw.Y()
        y = -raw.X()
        tmp = math.degrees(raw.rotation().Z())
        #tmp = tmp - 180
        tmp = move.normangle(tmp)
        tmp = 360 - tmp
        yaw = wpimath.geometry.Rotation2d.fromDegrees(tmp)
        trans = wpimath.geometry.Translation2d(x, y)
        print(self.name + "APRIL: Exiting")
        return (
            id,
            wpimath.geometry.Pose2d(trans, yaw),
            recent.getTimestampSeconds()
        )