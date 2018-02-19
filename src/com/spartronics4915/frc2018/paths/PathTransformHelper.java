package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class PathTransformHelper
{
    public static ArrayList<Waypoint> mirrorWaypointsAboutAxis(ArrayList<Waypoint> waypoints, boolean mirrorX, boolean mirrorY)
    {
        for (Waypoint w : waypoints)
        {
            w.position = mirrorTranslationAboutAxis(w.position, mirrorX, mirrorY);
        }
        return waypoints;
    }
    
    public static RigidTransform2d mirrorRigidTransformAboutAxis(RigidTransform2d rt, boolean mirrorX, boolean mirrorY)
    {
        rt.setTranslation(mirrorTranslationAboutAxis(rt.getTranslation(), mirrorX, mirrorY));
        return rt;
    }
    
    public static Translation2d mirrorTranslationAboutAxis(Translation2d translation, boolean mirrorX, boolean mirrorY)
    {
        if (mirrorX)
            translation.setX(Constants.kFieldDimensionTranslation.x() - translation.x());
        
        if (mirrorY)
            translation.setY(Constants.kFieldDimensionTranslation.y() - translation.y());
        
        return translation;
    }
}
