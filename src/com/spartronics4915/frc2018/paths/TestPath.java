package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class TestPath implements PathContainer
{

    @Override
    public Path buildPath()
    {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(50, 50, 0, 0));
        sWaypoints.add(new Waypoint(90, 50, 30, 50));
        sWaypoints.add(new Waypoint(90, 90, 0, 50));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose()
    {
        return new RigidTransform2d(new Translation2d(50, 50), Rotation2d.fromDegrees(90.0)); // XXX: I have no idea why this needs to be 90 instead of 180, but if it isn't things get reversed
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":50,"y":50},"speed":0,"radius":0,"comment":""},{"position":{"x":50,"y":90},"speed":50,"radius":20,"comment":""},{"position":{"x":90,"y":90},"speed":30,"radius":20,"comment":""},{"position":{"x":90,"y":130},"speed":30,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: TestPath
}
