package com.spartronics4915.frc2018.paths;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;

public class DriveToFarSwitchFromAPath extends DriveToFarSwitchFromCPath
{
    @Override
    public Path buildPath()
    {
        return PathBuilder.buildPathFromWaypoints(super.getWaypoints());
//        for (Waypoint w : super.getWaypoints())
//        {
//            
//        }
    }
    
    @Override
    public RigidTransform2d getStartPose()
    {
        return new RigidTransform2d(Constants.kFieldHeightTranslation.translateBy(super.getStartPose().getTranslation().inverse()),
                super.getStartPose().getRotation()); 
    }
}
