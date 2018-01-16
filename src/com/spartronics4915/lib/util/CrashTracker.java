package com.spartronics4915.lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't
 * roll over
 */
public class CrashTracker
{

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    public static int sVerbosity = 0; // 0: notices and above,  1: info and above, 2: all

    public static void logRobotStartup()
    {
        logMarker("robot startup");
    }

    public static void logRobotConstruction()
    {
        logMarker("robot construction");
    }

    public static void logRobotInit()
    {
        logMarker("robot init");
    }

    public static void logTeleopInit()
    {
        logMarker("teleop init");
    }

    public static void logAutoInit()
    {
        logMarker("auto init");
    }

    public static void logDisabledInit()
    {
        logMarker("disabled init");
    }

    public static void logThrowableCrash(Throwable throwable)
    {
        logMarker("Exception", throwable);
    }
    
    public static void error(String m)
    {
        logMarker("ERROR   " + m);
    }
    
    public static void warning(String m)
    {
        logMarker("WARNING " + m);        
    }
    
    public static void notice(String m)
    {
        logMarker("NOTICE  " + m);                
    }
    
    public static void info(String m)
    {
        if(sVerbosity > 0)
        {
            printMarker("INFO    " + m);
        }
    }

    public static void debug(String m)
    {
        if(sVerbosity > 1)
        {
            printMarker("DEBUG    " + m);
        }
    }
    
    private static void logMarker(String mark)
    {
        logMarker(mark, null);
    }
    
    private static void printMarker(String mark)
    {
        System.out.print(mark);        
    }

    private static void logMarker(String mark, Throwable nullableException)
    {
        System.out.print(mark);
        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true)))
        {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

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
