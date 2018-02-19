package com.spartronics4915.frc2018.paths;

import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;

/**
 * Interface containing all information necessary for a path including the Path
 * itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer
{

    Path buildPath();

    List<Waypoint> getWaypoints();
    
    RigidTransform2d getStartPose();

    boolean isReversed();
}
