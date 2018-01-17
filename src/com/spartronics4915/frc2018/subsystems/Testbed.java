package com.spartronics4915.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.drivers.CANTalon;
import com.spartronics4915.lib.util.drivers.CANTalonFactory;

/**
 * A stub of a subsystem for learning purposes.
 */
public class Testbed extends Subsystem
{

    // the keyword, 'static', should only be used if you know what you're doing and, even then, sparingly.
    private static Testbed sInstance = null;

    public static Testbed getInstance()
    {
        // Testbed is a singleton, meaning that only one object of the class
        // is ever instantiated.  Note that our constructor is private.
        if (sInstance == null)
        {
            sInstance = new Testbed();
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

    private CANTalon mMotor = null;
    private SystemState mSystemState = SystemState.IDLING;
    private WantedState mWantedState = WantedState.IDLE;

    private Testbed()
    {
        // Instantiate member variables (motors, etc...) here.
        CANProbe canProbe = CANProbe.getInstance();
        boolean success = false;
        if(canProbe.validateSRXId(Constants.kTestbedMotorId))
        {
            mMotor = CANTalonFactory.createDefaultTalon(Constants.kTestbedMotorId);
            mMotor.changeControlMode(ControlMode.PercentOutput);
            success = true;
        }
        else
        {
            this.logWarning("can't find testbed motor");
        }
        logInitialized(success);
    }

    // Any public method that isn't @Overriding an abstract method on the Subsystem superclass
    // MUST BE SYNCHRONIZED (because it's called from an Action in another thread).

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Testbed.this)
            {
                mSystemState = SystemState.IDLING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Testbed.this)
            {
                SystemState newState;
                switch (mSystemState)
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
                    logInfo("Testbed state from " + mSystemState + " to " + newState);
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
        }

    }; // end of assignment to mLoop

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
         * This methods overrides the abstract method of the same name in the
         * superclass (Subsystem). That just means that every Subsystem needs
         * to implement outputToSmartDashboard method.
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
        if(mMotor != null)
            mMotor.set(percentOutput);
    }

    /** describes the steps needed to progress from the current
     *  state to the wanted state.  Here, in open loop mode, we
     *  presume that the transition is instantaneous.
     * @return the current state
     */
    private SystemState defaultStateTransfer()
    {
        switch (mWantedState)
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
        runOpenLoop(1.0);
        if (mWantedState == WantedState.REVERSE_INTAKE)
        {
            return SystemState.IDLING;
        }
        return defaultStateTransfer();
    }

    private SystemState handleReverseIntake()
    {
        runOpenLoop(-1.0);
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
