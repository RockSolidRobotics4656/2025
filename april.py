from typing import *
import photonlibpy

camera = photonlibpy.PhotonCamera("testing")
count = 0


def test(telem):
    print("April Update")
    camera.setLEDMode(photonlibpy.photonCamera.VisionLEDMode.kBlink)
    result = camera.getLatestResult()
    telem.putNumber("Timestamp", result.getTimestampSeconds())
    global count
    telem.putNumber("Count", count)
    count += 1
    telem.putBoolean("Has Targets", result.hasTargets())
    best = result.getBestTarget()
    telem.putNumber("Tag ID", -1)
    if best:
        tagid = best.getFiducialId()
        telem.putNumber("Tag ID", tagid)
        pose = best.getBestCameraToTarget()
        telem.putNumber("x", pose.X())
        telem.putNumber("y", pose.Y())
        telem.putNumber("z", pose.Z())
        telem.putNumber("yaw", pose.rotation().Z() / 3.14 * 180)
