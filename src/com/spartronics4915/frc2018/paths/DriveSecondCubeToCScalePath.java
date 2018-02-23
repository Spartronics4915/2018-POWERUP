package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveSecondCubeToCScalePath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveSecondCubeToCScalePath()
    {
        sWaypoints.add(new Waypoint(213,90,0,60));
        sWaypoints.add(new Waypoint(253,73,20,60));
        sWaypoints.add(new Waypoint(283,73,0,60));

    }

    @Override
    public Path buildPath()
    {
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public List<Waypoint> getWaypoints()
    {
        return sWaypoints;
    }

    @Override
    public RigidTransform2d getStartPose()
    {
        return new RigidTransform2d(new Translation2d(213, 90), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":213,"y":90},"speed":60,"radius":0,"comment":""},{"position":{"x":253,"y":73},"speed":60,"radius":20,"comment":""},{"position":{"x":283,"y":73},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveSecondCubeToCScalePath
}