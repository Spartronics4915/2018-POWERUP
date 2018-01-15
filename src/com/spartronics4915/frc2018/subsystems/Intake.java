package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Looper;

/**
 * A stub of a subsystem for learning purposes.
 */
public class Intake extends Subsystem
{
    // The two fields below are the only time you should use the static
    // keyword without knowing what it means.
    private static Intake sInstance = null;
    
    public static Intake getInstance() {
        // Intake is a singleton, meaning that only one object of the class
        // is ever instantiated.
        if (sInstance == null)
        {
            sInstance = new Intake();
        }
        return sInstance;
    }
    
    private Intake()
    {
        // Instantiate member variables (motors, etc...) here.
    }
    
    // Any public method that isn't @Overriding an abstract method on the Subsystem superclass
    // MUST BE SYNCHRONIZED (because it's called from an Action in another thread).
    
    @Override
    public void outputToSmartDashboard()
    {
        /*
         * If we want to put a graph or visualization for debug purposes on the
         * dashboard, you would output information for that visualization here.
         * 
         * This methods overrides the abstract method of the same name in the
         * superclass (Subsystem). That just means that every Subsystem needs
         * to have an outputToSmartDashboard method.
         */
    }

    // stop should also be synchronized, because it's also called from another thread
    @Override
    public synchronized void stop()
    {
        /*
         * This is here for motor safety, so that when we go into disabled mode,
         * the robot actually stops. You should stop your motors here.
         */
    }

    @Override
    public void zeroSensors()
    {
    }
    
    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
    }

}
