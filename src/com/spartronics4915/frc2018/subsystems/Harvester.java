package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The harvester is a set of two collapsible rollers that pull in and hold
 * a cube. This cube is then in a position where it can be picked up by the
 * articulated grabber.
 */
public class Harvester extends Subsystem
{

    private static Harvester sInstance = null;

    public static Harvester getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new Harvester();
        }
        return sInstance;
    }

    public enum SystemState
    {
        CLOSING,
        OPENING,
        HARVESTING,
        EJECTING,
        HUGGING,
        PREHARVESTING,
        DISABLING,
    }

    public enum WantedState
    {
        CLOSE,
        OPEN,
        HARVEST,
        EJECT,
        HUG,
        PREHARVEST,
        DISABLE,
    }

    private SystemState mSystemState = SystemState.DISABLING;
    private WantedState mWantedState = WantedState.DISABLE;
    private DigitalInput mLimitSwitchCubeHeld = null;
    private DigitalInput mLimitSwitchEmergency = null;
    private Solenoid mSolenoid = null;
    private TalonSRX4915 mMotorRight = null;
    private TalonSRX4915 mMotorLeft = null;

    // Actuators and sensors should be initialized as private members with a value of null here

    private Harvester()
    {
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false

        mLimitSwitchCubeHeld = new DigitalInput(Constants.kHarvesterCubeHeldLimitSwitchId);// change value of Limit Switch
        mLimitSwitchEmergency = new DigitalInput(Constants.kHarvesterEmergencyLimitSwitchId); // changes value of Limit Switch
        mSolenoid = new Solenoid(Constants.kHarvesterSolenoidId); // Changes value of Solenoid
        mMotorRight = TalonSRX4915Factory.createDefaultMotor(Constants.kHarvesterRightMotorId); // change value of motor
        mMotorLeft = TalonSRX4915Factory.createDefaultMotor(Constants.kHarvesterLeftMotorId); // change value of motor
        mMotorLeft.setInvert();
        //mMotorRight.configOutputPower(true, 0.5, 0, 0.3, 0, -0.3);
        //mMotorLeft.configOutputPower(true, 0.5, 0, 0.3, 0, -0.3);
        
        if (!mMotorRight.isValid())
        {
            logError("Right Motor is invalid");
            success = false;
        }
        if (!mMotorLeft.isValid())
        {
            logError("Left Motor is invalid");
            success = false;
        }
        if (!Util.validateSolenoid(mSolenoid))
        {
            logError("Solenoid is invalid");
            success = false;
        }

        logInitialized(success);
    }

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Harvester.this)
            {
                mSystemState = SystemState.CLOSING;
                
                if (mSystemState == SystemState.DISABLING)
                    mWantedState = WantedState.CLOSE;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Harvester.this)
            {
                SystemState newState; // calls the wanted handle case for the given systemState
                switch (mSystemState)
                {
                    case CLOSING:
                        newState = handleClosing();
                        break;
                    case OPENING:
                        newState = handleOpening();
                        break;
                    case HARVESTING:
                        newState = handleHarvesting();
                        break;
                    case EJECTING:
                        newState = handleEjecting();
                        break;
                    case HUGGING:
                        newState = handleHugging();
                        break;
                    case PREHARVESTING:
                        newState = handlePreharvesting();
                    case DISABLING:
                        newState = handleClosing();
                    default:
                        newState = handleClosing();
                }
                if (newState != mSystemState)
                {
                    logInfo("Harvester state from " + mSystemState + "to" + newState);
                    dashboardPutState(mSystemState.toString());
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (Harvester.this)
            {
                stop();
            }
        }

    };

    private SystemState defaultStateTransfer() // transitions the systemState given what the wantedState is
    {

        switch (mWantedState)
        {
            case CLOSE:
                return SystemState.CLOSING;
            case OPEN:
                return SystemState.OPENING;
            case HARVEST:
                return SystemState.HARVESTING;
            case EJECT:
                return SystemState.EJECTING;
            case HUG:
                return SystemState.HUGGING;
            case PREHARVEST:
                return SystemState.PREHARVESTING;
            default:
                return mSystemState;

        }
    }

    private SystemState handleClosing()
    {
        //motors off and bars in
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.OPEN)
        {
            mSolenoid.set(true);
        }
            mMotorLeft.set(0.0);
            mMotorRight.set(0.0);
            return defaultStateTransfer(); // all defaultStateTransfers return the wanted state
    }

    private SystemState handleOpening()
    {
        //motors off and bars out
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.HARVEST)
        {
            mSolenoid.set(false);
        }
            mMotorLeft.set(0.0);
            mMotorRight.set(0.0);
            return defaultStateTransfer();
    }

    private SystemState handlePreharvesting()
    {
        //motors on and bars out
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.HARVEST)
        {
            mSolenoid.set(true);
        }
            mMotorLeft.set(1.0);
            mMotorRight.set(1.0);
            return defaultStateTransfer();
    }
    private SystemState handleHarvesting()
    {
        //motors on forward and bars closing, hug when cube is gone
        // You should probably be transferring state and controlling actuators in here
        if (!mLimitSwitchCubeHeld.get()) //THIS WILL PROBABLY CHANGE TO SENSOR
        {
            setWantedState(WantedState.HUG); // checks if cube is in the robot and will transitions to hugging when the cube is fully in
        }
        if (mWantedState != WantedState.CLOSE)
        {
            mSolenoid.set(false);
        }
            mMotorLeft.set(1.0);
            mMotorRight.set(1.0);
            return defaultStateTransfer();
    }

    private SystemState handleEjecting()
    {
        //motors in reverse and bars closing, close when cube is gone
        if (mLimitSwitchEmergency.get()) //checks if we have reached an emergency state, and will transition to open when it reaches emergency
        {
            mSolenoid.set(false);
            setWantedState(WantedState.OPEN);
        }
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState != WantedState.HUG)
        {
            mSolenoid.set(false);
        }
            mMotorLeft.set(-1.0);
            mMotorRight.set(-1.0);
            return defaultStateTransfer();
    }

    private SystemState handleHugging()
    {
        //motors off and bars closing go to closed when cube is gone
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState != WantedState.CLOSE)
        {
            mSolenoid.set(false);
        }
            mMotorLeft.set(0.0);
            mMotorRight.set(0.0);
            return defaultStateTransfer();
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
        dashboardPutWantedState(mWantedState.toString());
    }

    @Override
    public void outputToSmartDashboard()
    {
        dashboardPutState(mSystemState + "/SystemState");
        dashboardPutWantedState(mWantedState + "/WantedState");
        dashboardPutBoolean("/mSolenoid", mSolenoid.get());
        dashboardPutBoolean("/LimitSwitchCubeHeld", mLimitSwitchCubeHeld.get());
        dashboardPutBoolean("/LimitSwitchEmergency", mLimitSwitchEmergency.get());
        dashboardPutNumber("/MotorRight", mMotorRight.get());
        dashboardPutNumber("/MotorLeft", mMotorLeft.get());
    }

    @Override
    public synchronized void stop()
    {
        mSolenoid.set(false);
        mMotorLeft.set(0.0);
        mMotorRight.set(0.0);
        mSystemState = SystemState.DISABLING;
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
}
