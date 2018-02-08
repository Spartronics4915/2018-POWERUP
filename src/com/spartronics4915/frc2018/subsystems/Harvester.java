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
    }

    public enum WantedState
    {
        CLOSE,
        OPEN,
        HARVEST,
        EJECT,
        HUG,
    }

    private SystemState mSystemState = SystemState.CLOSING;
    private WantedState mWantedState = WantedState.CLOSE;
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
        mMotorRight.configOutputPower(true, 0.5, 0, 0.3, 0, -0.3);
        mMotorLeft.configOutputPower(true, 0.5, 0, 0.3, 0, -0.3);
        
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
            default:
                return mSystemState;

        }
    }

    private SystemState handleClosing()
    {

        //motors off and bars in
        mSolenoid.set(false);
        mMotorLeft.set(0.0);
        mMotorRight.set(0.0);

        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.OPEN)
        {
            return defaultStateTransfer(); // all defaultStateTransfers return the wanted state
        }
        else
        {
            return SystemState.CLOSING;
        }

    }

    private SystemState handleOpening()
    {
        //motors off and bars out
        mSolenoid.set(true);
        mMotorLeft.set(1.0);
        mMotorRight.set(1.0);

        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.HARVEST)
        {
            return defaultStateTransfer();
        }
        else
        {
            return SystemState.OPENING;
        }
    }

    private SystemState handleHarvesting()
    {
        //motors on forward and bars closing, hug when cube is gone
        mSolenoid.set(false);
        mMotorLeft.set(1.0);
        mMotorRight.set(1.0);

        // You should probably be transferring state and controlling actuators in here
        if (!mLimitSwitchCubeHeld.get())
        {
            setWantedState(WantedState.HUG); // checks if cube is in the robot and will transitions to hugging when the cube is fully in
        }
        if (mWantedState == WantedState.HUG || mWantedState == WantedState.EJECT
                || mWantedState == WantedState.OPEN)
        {
            logError("Set WantedState to hug");
            return defaultStateTransfer();
        }
        else
        {
            logError("Set WantedState to harvesting");
            return SystemState.HARVESTING;
        }
    }

    private SystemState handleEjecting()
    {
        //motors in reverse and bars closing, close when cube is gone
        mSolenoid.set(false);
        mMotorLeft.set(-1.0);
        mMotorRight.set(-1.0);

        if (mLimitSwitchEmergency.get()) //checks if we have reached an emergency state, and will transition to open when it reaches emergency
        {
            setWantedState(WantedState.OPEN);
        }

        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.OPEN || mWantedState == WantedState.CLOSE
                || mWantedState == WantedState.HARVEST)
        {
            return defaultStateTransfer();
        }
        else
        {
            return SystemState.EJECTING;
        }
    }

    private SystemState handleHugging()
    {
        //motors off and bars closing go to closed when cube is gone
        mSolenoid.set(false);
        mMotorLeft.set(0.0);
        mMotorRight.set(0.0);

        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.HARVEST || mWantedState == WantedState.EJECT
                || mWantedState == WantedState.OPEN)
        {
            return defaultStateTransfer();
        }
        else
        {
            return SystemState.HUGGING;
        }
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
        dashboardPutWantedState(mWantedState.toString());
    }

    @Override
    public void outputToSmartDashboard()
    {
        SmartDashboard.putString(this.getName() + "/SystemState", this.mSystemState + "");
        SmartDashboard.putString(this.getName() + "/WantedState", this.mWantedState + "");
        SmartDashboard.putString(this.getName() + "/mSolenoid", this.mSolenoid + "");
        SmartDashboard.putString(this.getName() + "/LimitSwitchCubeHeld",
                this.mLimitSwitchCubeHeld.get() + "");
        SmartDashboard.putString(this.getName() + "/LimitSwitchEmergency",
                this.mLimitSwitchEmergency.get() + "");
        SmartDashboard.putString(this.getName() + "/MotorRight", this.mMotorRight.get() + "");
        SmartDashboard.putString(this.getName() + "/MotorLeft", this.mMotorLeft.get() + "");
    }

    @Override
    public synchronized void stop()
    {
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
