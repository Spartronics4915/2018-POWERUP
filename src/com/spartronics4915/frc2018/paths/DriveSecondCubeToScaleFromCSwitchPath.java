package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveSecondCubeToScaleFromCSwitchPath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveSecondCubeToScaleFromCSwitchPath()
    {
        sWaypoints.add(new Waypoint(213,92,0,60));
        sWaypoints.add(new Waypoint(263,78,10,60));
        sWaypoints.add(new Waypoint(283,78,0,60));

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
        return new RigidTransform2d(new Translation2d(213, 92), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":213,"y":92},"speed":60,"radius":0,"comment":""},{"position":{"x":263,"y":78},"speed":60,"radius":10,"comment":""},{"position":{"x":283,"y":78},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveSecondCubeToScaleFromCSwitchPath
}