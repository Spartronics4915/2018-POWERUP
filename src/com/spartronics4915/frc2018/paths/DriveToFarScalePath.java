package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveToFarScalePath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,45,0,0));
        sWaypoints.add(new Waypoint(238,45,70,60));
        sWaypoints.add(new Waypoint(238,250,30,40));
        sWaypoints.add(new Waypoint(318,290,20,10));
        sWaypoints.add(new Waypoint(318,270,0,0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 45), Rotation2d.fromDegrees(90.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":45},"speed":0,"radius":0,"comment":""},{"position":{"x":238,"y":45},"speed":60,"radius":70,"comment":""},{"position":{"x":238,"y":250},"speed":40,"radius":30,"comment":""},{"position":{"x":318,"y":290},"speed":10,"radius":20,"comment":""},{"position":{"x":318,"y":270},"speed":0,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: DriveToFarScalePath
}