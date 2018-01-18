package com.spartronics4915.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.drivers.CANTalon4915;
import com.spartronics4915.lib.util.drivers.CANTalonFactory;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private SystemState mSystemState = SystemState.IDLING;
    private WantedState mWantedState = WantedState.IDLE;

    // actuators and sensors
    private CANTalon4915 mMotor1 = null;
    private CANTalon4915 mMotor2 = null;
    private DigitalInput mLimitSwitch = null; // invoke .get() to read
    private AnalogInput mPotentiometer = null;
    private Relay mLightSwitch = null;
    private Servo mServo = null; // Servo subclasses PWM

    private Testbed()
    {
        // Instantiate member variables (motors, etc...) here.
        CANProbe canProbe = CANProbe.getInstance();
        boolean success = true;
        if(canProbe.validateSRXId(Constants.kTestbedMotor1Id))
        {
            mMotor1 = CANTalonFactory.createDefaultTalon(Constants.kTestbedMotor1Id);
            mMotor1.changeControlMode(ControlMode.PercentOutput);
        }
        else
        {
            logWarning("can't find motor 1, id:" + Constants.kTestbedMotor1Id);
            success = false;
        }

        if(canProbe.validateSRXId(Constants.kTestbedMotor2Id))
        {
            mMotor2 = CANTalonFactory.createDefaultTalon(Constants.kTestbedMotor2Id);
            mMotor2.changeControlMode(ControlMode.PercentOutput);
        }
        else
        {
            logWarning("can't find motor 2, id:" + Constants.kTestbedMotor2Id);
            success = false;
        }

        mLimitSwitch = new DigitalInput(Constants.kTestbedLimitSwitchId);
        mPotentiometer = new AnalogInput(Constants.kTestbedPotentiometerId);
        mLightSwitch = new Relay(Constants.kTestbedLightSwitchId);
        mServo = new Servo(Constants.kTestbedServoId);

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
        /* put useful state to the smart dashboard
         * 
         */
        SmartDashboard.putString("Testbed_Relay", mLightSwitch.get().getPrettyValue());
        SmartDashboard.putBoolean("Testbed_LimitSwitch", mLimitSwitch.get());
        SmartDashboard.putNumber("Testbed_Potentiometer", mPotentiometer.getValue());
        SmartDashboard.putNumber("Testbed_Servo", mServo.get());
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
        if(mMotor1 != null)
            mMotor1.set(percentOutput);
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
