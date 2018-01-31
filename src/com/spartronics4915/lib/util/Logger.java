package com.spartronics4915.lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't
 * roll over
 */
public class Logger
{

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    public static int sVerbosity = 0; // 0: notices and above,  1: info and above, 2: all
    private static final DateFormat s_dateFormat = new SimpleDateFormat("hh:mm:ss"); 

    public static void logRobotStartup()
    {
        notice("robot startup");
    }

    public static void logRobotConstruction()
    {
        notice("robot construction");
    }

    public static void logRobotInit()
    {
        notice("robot init");
    }
    
    public static void logTeleopInit()
    {
        notice("teleop init");
    }

    public static void logAutoInit()
    {
        notice("auto init");
    }

    public static void logDisabledInit()
    {
        notice("disabled init");
    }

    public static void logThrowableCrash(Throwable throwable)
    {
        logMarker(getTimeStamp() + "Exception", throwable);
    }

    public static void error(String m)
    {
        logMarker(getTimeStamp() + "ERROR   " + m);
    }

    public static void warning(String m)
    {
        logMarker(getTimeStamp() + "WARNING " + m);
    }

    public static void notice(String m)
    {
        logMarker(getTimeStamp() + "NOTICE  " + m);
    }

    public static void info(String m)
    {
        if (sVerbosity > 0)
        {
            printMarker(getTimeStamp() + "INFO    " + m);
        }
    }

    public static void debug(String m)
    {
        if (sVerbosity > 1)
        {
            printMarker(getTimeStamp() + "DEBUG    " + m);
        }
    }
    
    private static String getTimeStamp()
    {
        Date now = new Date();
        String nowstr = s_dateFormat.format(now);
        return nowstr;
    }

    private static void logMarker(String mark)
    {
        logMarker(mark, null);
    }

    private static void printMarker(String mark)
    {
        System.out.println(mark);
    }

    private static void logMarker(String mark, Throwable nullableException)
    {
        System.out.println(mark);
        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true)))
        {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            if (nullableException != null)
            {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }
            writer.println();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
