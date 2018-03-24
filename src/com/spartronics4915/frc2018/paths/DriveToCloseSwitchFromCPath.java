package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveToCloseSwitchFromCPath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveToCloseSwitchFromCPath()
    {
        sWaypoints.add(new Waypoint(18,45,0,0));
        sWaypoints.add(new Waypoint(170,45,15,60));
        sWaypoints.add(new Waypoint(170,69,0,30));

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
        return new RigidTransform2d(new Translation2d(18, 45), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":18,"y":45},"speed":0,"radius":0,"comment":""},{"position":{"x":170,"y":45},"speed":60,"radius":15,"comment":""},{"position":{"x":170,"y":69},"speed":30,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveToCloseSwitchFromCPath
}