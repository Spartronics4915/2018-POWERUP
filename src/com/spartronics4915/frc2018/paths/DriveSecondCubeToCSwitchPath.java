package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveSecondCubeToCSwitchPath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveSecondCubeToCSwitchPath()
    {
        sWaypoints.add(new Waypoint(270,40,0,60));
        sWaypoints.add(new Waypoint(239,92,9,60));
        sWaypoints.add(new Waypoint(226,92,0,60, "acquirecube"));
        sWaypoints.add(new Waypoint(213,92,0,60));

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
        return new RigidTransform2d(new Translation2d(270, 40), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":270,"y":40},"speed":60,"radius":0,"comment":""},{"position":{"x":239,"y":92},"speed":60,"radius":9,"comment":""},{"position":{"x":226,"y":92},"speed":60,"radius":0,"comment":""},{"position":{"x":213,"y":92},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveSecondCubeToCSwitchPath
}