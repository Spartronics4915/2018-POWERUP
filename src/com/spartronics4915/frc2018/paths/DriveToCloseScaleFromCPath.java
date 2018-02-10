package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveToCloseScaleFromCPath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,45,0,0));
        sWaypoints.add(new Waypoint(40,45,0,60));
        sWaypoints.add(new Waypoint(324,40,0,60));

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
	// WAYPOINT_DATA: [{"position":{"x":18,"y":45},"speed":0,"radius":0,"comment":""},{"position":{"x":40,"y":45},"speed":60,"radius":0,"comment":""},{"position":{"x":324,"y":40},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: DriveToCloseScaleFromCPath
}