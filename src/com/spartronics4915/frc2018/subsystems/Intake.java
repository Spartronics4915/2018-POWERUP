package com.spartronics4915.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.drivers.CANTalon;
import com.spartronics4915.lib.util.drivers.CANTalonFactory;

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
    
    public enum SystemState
    {
        FORWARD_INTAKING,
        REVERSE_INTAKING,
        IDLING,
    }
    public enum WantedState
    {
        FORWARD_INTAKE,
        REVERSE_INTAKE,
        IDLE,
    }

    private CANTalon mMotor;
    private SystemState mSystemState = SystemState.IDLING;
    private WantedState mWantedState = WantedState.IDLE;
    
    private Intake()
    {
        // Instantiate member variables (motors, etc...) here.
        mMotor = CANTalonFactory.createDefaultTalon(Constants.kIntakeId);
        
        mMotor.changeControlMode(ControlMode.PercentOutput);
    }
    
    private Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp)
        {
            synchronized(Intake.this)
            {
                mSystemState = SystemState.IDLING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(Intake.this)
            {
                SystemState newState;
                switch(mSystemState)
                {
                    case FORWARD_INTAKING:
                        newState = handleForwardIntake();
                        break;
                    case REVERSE_INTAKING:
                        newState = handleReverseIntake();
                        break;
                    case IDLING:
                        newState = handleIdle();
                        break;
                    default:
                        newState = SystemState.IDLING;
                        break;
                }
                if (newState != mSystemState)
                {
                    System.out.println("Intake state from " + mSystemState + " to " + newState);
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
        }
        
    };
    
    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
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
        enabledLooper.register(mLoop);
    }
    
    private void runOpenLoop(double percentOutput)
    {
        mMotor.set(percentOutput);
    }
    
    private SystemState defaultStateTransfer()
    {
        switch(mWantedState)
        {
            case FORWARD_INTAKE:
                return SystemState.FORWARD_INTAKING;
            case REVERSE_INTAKE:
                return SystemState.REVERSE_INTAKING;
            case IDLE:
                return SystemState.IDLING;
            default:
                return SystemState.IDLING;
        }
    }
    
    private SystemState handleForwardIntake()
    {
        mMotor.set(1.0);
        if (mWantedState == WantedState.REVERSE_INTAKE)
        {
            return SystemState.IDLING;
        }
        return defaultStateTransfer();
    }
    
    private SystemState handleReverseIntake()
    {
        mMotor.set(-1.0);
        if (mWantedState == WantedState.REVERSE_INTAKE)
        {
            return SystemState.IDLING;
        }
        return defaultStateTransfer();
    }
    
    private SystemState handleIdle()
    {
        return defaultStateTransfer();
    }

}
