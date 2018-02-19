package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveToCloseSwitchFromBPath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveToCloseSwitchFromBPath()
    {
        sWaypoints.add(new Waypoint(18,166,0,0));
        sWaypoints.add(new Waypoint(55,166,10,60));
        sWaypoints.add(new Waypoint(95,206,10,60));
        sWaypoints.add(new Waypoint(125,206,0,40)); //Intentional overshoot

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
        return new RigidTransform2d(new Translation2d(18, 166), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":18,"y":166},"speed":0,"radius":0,"comment":""},{"position":{"x":55,"y":166},"speed":60,"radius":10,"comment":""},{"position":{"x":95,"y":206},"speed":60,"radius":10,"comment":""},{"position":{"x":125,"y":206},"speed":40,"radius":0,"comment":"Intentional overshoot"}]
    // IS_REVERSED: false
    // FILE_NAME: DriveToCloseSwitchFromBPath
}