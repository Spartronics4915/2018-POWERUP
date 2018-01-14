package com.spartronics4915.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.drivers.CANTalon;
import com.spartronics4915.lib.util.drivers.CANTalonFactory;

/**
 * The Example subsystem controls two motors motor based upon the input from
 * a limit switch and the driver (in Teleop). The motors run in opposition
 * to each other, so the driver can just reverse the two motors. The limit
 * switch stops the motors.
 * 
 * This {@link Subsystem}'s methods can be called from {@link Superstructure}
 * as a result of a change of state in {@link Superstructure} or from an
 * {@link Action} directly (an action could interact with {@link Superstructure}
 * too.
 * 
 * @author declan
 * @see Superstructure
 * @see ExampleAction
 * @see ExampleMode
 */
public class Example extends Subsystem
{
    // The two fields below are the only time you should use the static
    // keyword without knowing what it means.
    private static Example sInstance = null;
    
    public static Example getInstance() {
        // Example is a singleton, meaning that only one object of the class
        // is ever instantiated.
        if (sInstance == null)
        {
            sInstance = new Example();
        }
        return sInstance;
    }
    
    // You can add state enums here
    
    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp)
        {
            // Nothing needs to happen here yet
            
        }
        
        @Override
        public void onLoop(double timestamp) {
            synchronized (Example.this) {
                // State transitions can happen here
                if (mPrimaryMotor.isFwdLimitSwitchClosed() ||
                        mSecondaryMotor.isFwdLimitSwitchClosed()) {
                    setOff();
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            // Nothing needs to happen here yet
            
        } 
    };
    
    // Hardware
    private CANTalon mPrimaryMotor, mSecondaryMotor;
    
    private Example()
    {
        // You should instantiate your motors in the constructor of your subsystem
        mPrimaryMotor = CANTalonFactory.createDefaultTalon(Constants.kPrimaryExampleId);
        mSecondaryMotor = CANTalonFactory.createDefaultTalon(Constants.kSecondaryExampleId);
        
        // Set up limit switches
        mPrimaryMotor.configFwdLimitSwitchNormallyOpen(false);
        mSecondaryMotor.configFwdLimitSwitchNormallyOpen(false);
        
        // Use an open loop control mode
        mPrimaryMotor.changeControlMode(ControlMode.PercentOutput);
        mSecondaryMotor.changeControlMode(ControlMode.PercentOutput);
    }
    
    private void setOpenLoop(double speed) {
        // We avoid code duplication (which is bad) by having as single method for controlling motors
        mPrimaryMotor.set(ControlMode.PercentOutput, speed);
        mSecondaryMotor.set(ControlMode.PercentOutput, speed);
    }
    
    // Any method called from an Action MUST be synchronized
    public synchronized void setOn() {
        setOpenLoop(1.0);
    }
    
    // Any method called from an Action MUST be synchronized
    public synchronized void setOff() {
        setOpenLoop(0.0);
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

    // Any method called from an Action MUST be synchronized
    @Override
    public synchronized void stop()
    {
        /*
         * This is here for motor safety, so that when we go into disabled mode,
         * the robot actually stops. You should stop your motors here.
         */
        setOff();
    }

    @Override
    public void zeroSensors()
    {
        // Our only sensors are limit switches, which don't really need zeroing
    }
    
    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        // This is so the looper loop actually gets invoked
        enabledLooper.register(mLoop);
    }

}
