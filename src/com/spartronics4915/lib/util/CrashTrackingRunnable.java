package com.spartronics4915.lib.util;

/**
 * Runnable class with reports all uncaught throws to CrashTracker
 */
public abstract class CrashTrackingRunnable implements Runnable
{

    @Override
    public final void run()
    {
        try
        {
            runCrashTracked();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    public abstract void runCrashTracked();
}
