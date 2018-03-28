package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveSecondCubeToSwitchFromCScalePath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveSecondCubeToSwitchFromCScalePath()
    {
        sWaypoints.add(new Waypoint(324,53,0,40));
        sWaypoints.add(new Waypoint(265,53,20,40));
        sWaypoints.add(new Waypoint(245,92,13,40));
        sWaypoints.add(new Waypoint(228,92,0,40, "acquirecube"));
        sWaypoints.add(new Waypoint(213,92,0,40));

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
        return new RigidTransform2d(new Translation2d(324, 53), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":324,"y":53},"speed":40,"radius":0,"comment":""},{"position":{"x":265,"y":53},"speed":40,"radius":20,"comment":""},{"position":{"x":245,"y":92},"speed":40,"radius":13,"comment":""},{"position":{"x":228,"y":92},"speed":40,"radius":0,"comment":""},{"position":{"x":213,"y":92},"speed":40,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveSecondCubeToSwitchFromCScalePath
}