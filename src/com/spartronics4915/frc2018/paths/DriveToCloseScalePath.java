package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveToCloseScalePath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,45,0,0));
        sWaypoints.add(new Waypoint(50,45,3,60));
        sWaypoints.add(new Waypoint(238,22,53,60));
        sWaypoints.add(new Waypoint(318,30,18,60));
        sWaypoints.add(new Waypoint(318,53,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 45), Rotation2d.fromDegrees(90.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
    // WAYPOINT_DATA: [{"position":{"x":18,"y":45},"speed":0,"radius":0,"comment":""},{"position":{"x":50,"y":45},"speed":60,"radius":3,"comment":""},{"position":{"x":238,"y":22},"speed":60,"radius":53,"comment":""},{"position":{"x":318,"y":30},"speed":60,"radius":18,"comment":""},{"position":{"x":318,"y":53},"speed":20,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveToCloseScalePath
}