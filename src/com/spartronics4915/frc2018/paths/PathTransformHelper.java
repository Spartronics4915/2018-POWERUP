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
        ArrayList<Waypoint> newWaypoints = new ArrayList<Waypoint>();
        for (Waypoint w : waypoints)
        {
            newWaypoints.add(new Waypoint(mirrorTranslationAboutAxis(w.position, mirrorX, mirrorY), w.radius, w.speed));
        }
        return newWaypoints;
    }
    
    public static RigidTransform2d mirrorRigidTransformAboutAxis(RigidTransform2d rt, boolean mirrorX, boolean mirrorY)
    {
        rt.setTranslation(mirrorTranslationAboutAxis(rt.getTranslation(), mirrorX, mirrorY));
        return rt;
    }
    
    public static Translation2d mirrorTranslationAboutAxis(Translation2d oldTranslation, boolean mirrorX, boolean mirrorY)
    {
        Translation2d t = Translation2d.identity();
        if (mirrorX)
        {
            t.setX(Constants.kFieldWidthTranslation.translateBy(oldTranslation.inverse()).x());
        }
        else
            t.setX(oldTranslation.x());
        
        if (mirrorY)
        {
            t.setY(Constants.kFieldHeightTranslation.translateBy(oldTranslation.inverse()).y());
        }
        else
            t.setY(oldTranslation.y());
        
        Logger.notice("Transforming (X,Y): " + oldTranslation.toString() + " to: " + t.toString());
        
        return t;
    }
}
