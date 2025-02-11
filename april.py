from typing import *
import photonlibpy
import threading
import dataclasses
import time
import wpimath
import wpimath.geometry
import ncoms
import commands2

class VisionSystem:
    def __init__(self, name: str):
        super().__init__()
        self.camera = photonlibpy.PhotonCamera(name)

    def __call__(self) -> Optional[Tuple[int, wpimath.geometry.Pose2d, float]]:
        result = self._get()
        ncoms.coproc_tab.putBoolean("Connected", self.camera.isConnected())
        if result:
            ncoms.coproc_tab.putNumber("Update Status", result[2])
            ncoms.coproc_tab.putBoolean("HasTarget", True)
            ncoms.coproc_tab.putNumber("Tagid", result[0])
            pose = result[1]
            ncoms.coproc_tab.putNumber("posex", pose.X())
            ncoms.coproc_tab.putNumber("posey", pose.Y())
            ncoms.coproc_tab.putNumber("posea", pose.rotation().degrees())
            return result
        ncoms.coproc_tab.putBoolean("HasTarget", False)
        return None

    def _get(self) -> Optional[Tuple[int, wpimath.geometry.Pose2d, float]]:
        print("APRIL: Fetching Latest Result")
        recent = self.camera.getLatestResult()
        if not recent.hasTargets(): return
        print("APRIL: Acquiring Best Target")
        target = recent.getBestTarget()
        id = target.getFiducialId()
        print("APRIL: Calculating the pose")
        raw = target.getBestCameraToTarget()
        x = raw.Y()
        y = -raw.X()
        yaw = raw.rotation().Z() / 3.1415 * 180
        yaw -= 180
        while yaw < 0: yaw += 360
        yaw %= 360
        yaw = 360 - yaw
        trans = wpimath.geometry.Translation2d(x, y)
        rot = wpimath.geometry.Rotation2d.fromDegrees(yaw)
        print("APRIL: Exiting")
        return (
            id,
            wpimath.geometry.Pose2d(trans, rot),
            recent.getTimestampSeconds()
        )