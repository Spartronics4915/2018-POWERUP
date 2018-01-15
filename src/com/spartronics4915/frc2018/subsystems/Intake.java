package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.lib.util.drivers.CANTalonFactory;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spartronics4915.frc2018.Constants;

import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.drivers.CANTalon;

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
    
    private CANTalon mMotor;
    
    private Intake()
    {
        // Instantiate member variables (motors, etc...) here.
        mMotor = CANTalonFactory.createDefaultTalon(Constants.kIntakeId);
        mMotor.changeControlMode(ControlMode.PercentOutput);
    }
    
    
    
    // Any public method that isn't @Overriding an abstract method on the Subsystem superclass
    // MUST BE SYNCHRONIZED (because it's called from an Action in another thread).
    
    private void runOpenLoop(double percentOutput)
    {
        mMotor.set(percentOutput);
    }
    
    public synchronized void runForward()
    {
        runOpenLoop(1.0);
    }
    
    public synchronized void runRevese()
    {
        runOpenLoop(-1.0);
    }
    
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
        
        runOpenLoop(0.0);
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
