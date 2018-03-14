package com.spartronics4915.lib.util;

import java.util.List;

import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * Contains basic functions that are used often.
 */
public class Util
{

    /** Prevent this class from being instantiated. */
    private Util()
    {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude)
    {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max)
    {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, List<?> strings)
    {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i)
        {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1)
            {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    // check if a ~= b
    public static boolean epsilonEquals(double a, double b, double epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    // check if a < b
    public static boolean epsilonLessThan(double a, double b, double epsilon)
    {
        return (a + epsilon) < b;
    }

    // check if a > b
    public static boolean epsilonGreaterThan(double a, double b, double epsilon)
    {
        return (a - epsilon) > b;
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon)
    {
        boolean result = true;
        for (Double value_in : list)
        {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }
    
    public static String getGameSpecificMessage()
    {
        Timer t = new Timer();
        String result = "";
        DriverStation ds = DriverStation.getInstance();
        t.start();
        while(!t.hasPeriodPassed(3)) // wait for up to 3 sec
        {
            result = ds.getGameSpecificMessage();
            if(!result.equals(""))
                break;
            Timer.delay(0.1);
        }
        return result;
    }
    
    public static PathContainer truncatePathContainerUntilMarker(PathContainer pc, String marker) {
        boolean hasFoundMarker = false;
        for (Waypoint w : pc.getWaypoints()) {
            if (!hasFoundMarker)
                if (w.getMarker().equals(marker))
                    hasFoundMarker = true;
                else
                    pc.getWaypoints().remove(w);
        }
        return pc;
    }
}
