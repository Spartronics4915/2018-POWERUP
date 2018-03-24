package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveReverseToSecondCubeFromCSwitchPath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveReverseToSecondCubeFromCSwitchPath()
    {
        sWaypoints.add(new Waypoint(160,68,0,30));
        sWaypoints.add(new Waypoint(160,40,18,60, "openharvester"));
        sWaypoints.add(new Waypoint(270,40,0,60));

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
        return new RigidTransform2d(new Translation2d(160, 68), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return true;
    }
    // WAYPOINT_DATA: [{"position":{"x":160,"y":68},"speed":30,"radius":0,"comment":""},{"position":{"x":160,"y":40},"speed":60,"radius":18,"comment":""},{"position":{"x":270,"y":40},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: DriveReverseToSecondCubeFromCSwitchPath
}