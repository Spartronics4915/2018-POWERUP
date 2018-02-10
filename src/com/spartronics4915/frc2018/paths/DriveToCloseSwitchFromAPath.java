package com.spartronics4915.frc2018.paths;

import com.spartronics4915.lib.util.math.RigidTransform2d;

public class DriveToCloseSwitchFromAPath extends DriveToCloseSwitchFromCPath
{
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(super.getStartPose().getTranslation(), super.getStartPose().getRotation().inverse()); 
    }
    
    @Override
    public boolean isReversed() {
        return true;
    }
}
