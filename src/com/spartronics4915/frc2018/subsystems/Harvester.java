package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.drivers.LazySolenoid;
import com.spartronics4915.lib.util.drivers.SpartIRSensor;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * The harvester is a set of two collapsible rollers that pull in and hold
 * a cube. This cube is then in a position where it can be picked up by the
 * articulated grabber.
 */
public class Harvester extends Subsystem
{

    private static Harvester sInstance = null;
    
    private static final boolean kFloatingSolenoidEngaged = true;
    private static final boolean kGrabbingSolenoidEngaged = true;
    private static final double kFlipperSpeed = 1; // This is direction-independent
    private static final double kFlipperAllowedError = 20;
    private static final double kSlideDropOffset = 155; // In potentiometer ticks
    
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
        DISABLING,
        INTAKING,
        GRABBING,
        EJECTING,
        DEPLOYING,
        STOWING,
        SLIDE_DROPPING,
    }

    public enum WantedState
    {
        DISABLE,
        INTAKE,
        GRAB,
        EJECT,
        DEPLOY,
        STOW,
        SLIDE_DROP,
    }

    private SystemState mSystemState = SystemState.DISABLING; // We don't want to modify the state of any of the hardware when we start up
    private WantedState mWantedState = WantedState.DISABLE;
    private LazySolenoid mGrabbingSolenoid = null;
    private LazySolenoid mFloatingSolenoid = null;
    private AnalogInput mFlipperPotentiometer = null;
    private DigitalInput mReverseLimit = null;
    private DigitalInput mForwardLimit = null;
    private SpartIRSensor mCubeHeldSensor = null;
    private TalonSRX4915 mIntakeMotorRight = null;
    private TalonSRX4915 mIntakeMotorLeft = null;
    private TalonSRX4915 mFlipperMotor = null;
    private Timer mTimer;
    
    private double mReverseLimitPosition = 0;

    // Actuators and sensors should be initialized as private members with a value of null here

    private Harvester()
    {
        boolean success = true;

        try
        {
            mGrabbingSolenoid = new LazySolenoid(Constants.kGrabberSolenoidId);
            mFloatingSolenoid = new LazySolenoid(Constants.kGrabberSetupSolenoidId);
            mFlipperPotentiometer = new AnalogInput(Constants.kGrabberAnglePotentiometerId);
            mReverseLimit = new DigitalInput(Constants.kFlipperRevLimitSwitchId);
            mForwardLimit = new DigitalInput(Constants.kFlipperFwdLimitSwitchId);
            mCubeHeldSensor = new SpartIRSensor(Constants.kGrabberCubeDistanceRangeFinderId);
            mIntakeMotorRight = TalonSRX4915Factory.createDefaultMotor(Constants.kHarvesterRightMotorId); // change value of motor
            mIntakeMotorLeft = TalonSRX4915Factory.createDefaultMotor(Constants.kHarvesterLeftMotorId); // change value of motor
            mFlipperMotor = TalonSRX4915Factory.createDefaultMotor(Constants.kGrabberFlipperMotorId);
            mIntakeMotorRight.configOutputPower(true, 0.5, 0, 1, 0, -1);
            mIntakeMotorLeft.configOutputPower(true, 0.5, 0, 1, 0, -1);
            mIntakeMotorRight.setInverted(true);
            mTimer = new Timer();

            if (!mIntakeMotorRight.isValid())
            {
                logError("Right Motor is invalid");
                success = false;
            }
            if (!mIntakeMotorLeft.isValid())
            {
                logError("Left Motor is invalid");
                success = false;
            }
            if (!mGrabbingSolenoid.isValid())
            {
                logError("Grabbing solenoid is invalid");
                success = false;
            }
            if (!mFloatingSolenoid.isValid())
            {
                logError("Floating solenoid is invalid");
                success = false;
            }
        }
        catch (Exception e)
        {
            logError("Couldn't instantiate hardware objects");
            Logger.logThrowableCrash(e);
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

                if (mSystemState == SystemState.DISABLING)
                    mSystemState = SystemState.GRABBING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Harvester.this)
            {
                if (!mReverseLimit.get())
                    mReverseLimitPosition = mFlipperPotentiometer.getAverageValue();
                
                SystemState newState = SystemState.DISABLING; // calls the wanted handle case for the given systemState
                switch (mSystemState)
                {
                    case DISABLING:
                        mIntakeMotorRight.set(0);
                        mIntakeMotorLeft.set(0);
                        // Don't change the hardware in this state
                        newState = handleDisabling();
                        break;
                    case INTAKING:
                        setSolenoidsToFloating();
                        mIntakeMotorRight.set(1);
                        mIntakeMotorLeft.set(1);
                        newState = defaultStateTransfer();
                        break;
                    case GRABBING:
                        setSolenoidsToGrabbing();
                        mIntakeMotorRight.set(0);
                        mIntakeMotorLeft.set(0);
                        newState = defaultStateTransfer();
                        break;
                    case EJECTING:
                        setSolenoidsToFloating();
                        mIntakeMotorRight.set(-1);
                        mIntakeMotorLeft.set(-1);
                        newState = defaultStateTransfer();
                        break;
                    case DEPLOYING:
                        setSolenoidsToOpening();
                        flipperForward();
                        newState = defaultStateTransfer();
                        break;
                    case SLIDE_DROPPING:
                        boolean onTarget = flipperToTarget(mReverseLimitPosition + kSlideDropOffset);
                        if (onTarget) // We could have another state for this, but there's not benefit
                        {
                            setSolenoidsToOpening();
                        }
                        break;
                    case STOWING:
                        setSolenoidsToGrabbing();
                        flipperReverse();
                        if (mWantedState == WantedState.DEPLOY)
                            newState = defaultStateTransfer();
                        break;
                    default:
                        newState = handleDisabling();
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

    private SystemState handleDisabling()
    {
        return defaultStateTransfer(); // Don't touch the hardware in this state
    }
    
    private SystemState defaultStateTransfer() // transitions the systemState given what the wantedState is
    {

        switch (mWantedState)
        {
            case DISABLE:
                return SystemState.DISABLING;
            case INTAKE:
                return SystemState.INTAKING;
            case GRAB:
                return SystemState.GRABBING;
            case EJECT:
                return SystemState.EJECTING;
            case DEPLOY:
                return SystemState.DEPLOYING;
            case STOW:
                return SystemState.STOWING;
            case SLIDE_DROP:
                return SystemState.SLIDE_DROPPING;
            default:
                return mSystemState;
        }
    }
    
    private boolean flipperToTarget(double target)
    {
        if (Util.epsilonGreaterThan(mFlipperPotentiometer.getAverageValue(), target, kFlipperAllowedError))
            flipperForward();
        else if (Util.epsilonLessThan(mFlipperPotentiometer.getAverageValue(), target, kFlipperAllowedError))
            flipperReverse();
        else
        {
            mFlipperMotor.set(0);
            return true;
        }
        return false;
    }
    
    private void flipperForward()
    {
        if (mForwardLimit.get())
            mFlipperMotor.set(kFlipperSpeed);
        else
            mFlipperMotor.set(0);
    }
    
    private void flipperReverse()
    {
        if (mReverseLimit.get())
            mFlipperMotor.set(-kFlipperSpeed);
        else
            mFlipperMotor.set(0);
    }
    
    public WantedState getWantedState()
    {
        return mWantedState;
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
        dashboardPutWantedState(mWantedState.toString());
    }

    public boolean atTarget()
    {
        boolean t = false;
        switch (mSystemState)
        {
            case DISABLING:
                if (mWantedState == WantedState.DISABLE)
                    t = true;
            case INTAKING:
                if (mWantedState == WantedState.INTAKE)
                    t = true;
            case GRABBING:
                if (mWantedState == WantedState.GRAB)
                    t = true;
            case EJECTING:
                if (mWantedState == WantedState.EJECT)
                    t = true;
            case DEPLOYING:
                if (mWantedState == WantedState.DEPLOY
                     && !mForwardLimit.get())
                    t = true;
            case STOWING:
                if (mWantedState == WantedState.STOW
                     && !mReverseLimit.get())
                    t = true;
            case SLIDE_DROPPING:
                if (mWantedState == WantedState.SLIDE_DROP
                     && Util.epsilonEquals(mFlipperPotentiometer.getAverageValue(), mReverseLimitPosition + kSlideDropOffset, kFlipperAllowedError))
                    t = true;
            default:
                t = false;
                break;
        }
        return t;
    }

    private boolean isCubeHeld()
    {
        return mCubeHeldSensor.isTargetAcquired();
    }

    @Override
    public void outputToSmartDashboard()
    {
        if(!isInitialized()) return;
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("mGrabbingSolenoid", mGrabbingSolenoid.get());
        dashboardPutNumber("mFlipperPotentiometer", mFlipperPotentiometer.getAverageValue());
        dashboardPutBoolean("mReverseLimit", mReverseLimit.get());
        dashboardPutBoolean("mForwardLimit", mForwardLimit.get());
        dashboardPutBoolean("IRSensor CubeHeld", isCubeHeld());
        dashboardPutNumber("mIntakeMotorRight", mIntakeMotorRight.get());
        dashboardPutNumber("mIntakeMotorLeft", mIntakeMotorLeft.get());
        dashboardPutNumber("FlipperMotor", mFlipperMotor.get());
        dashboardPutNumber("Cube Distance: ", mCubeHeldSensor.getDistance());
    }

    @Override
    public synchronized void stop()
    {
        mSystemState = SystemState.DISABLING;
        mWantedState = WantedState.DISABLE;
        setSolenoidsToFloating();
        mIntakeMotorLeft.set(0.0);
        mIntakeMotorRight.set(0.0);
    }

    @Override
    public void zeroSensors()
    {
        if (!mReverseLimit.get())
            mReverseLimitPosition = mFlipperPotentiometer.getAverageValue();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    private void setSolenoidsToFloating()
    {
        mFloatingSolenoid.set(!kFloatingSolenoidEngaged);
        mGrabbingSolenoid.set(!kGrabbingSolenoidEngaged);
    }
    
    private void setSolenoidsToGrabbing()
    {
        mFloatingSolenoid.set(!kFloatingSolenoidEngaged);
        mGrabbingSolenoid.set(kGrabbingSolenoidEngaged);
    }
    
    private void setSolenoidsToOpening()
    {
        mFloatingSolenoid.set(kFloatingSolenoidEngaged);
        mGrabbingSolenoid.set(!kGrabbingSolenoidEngaged);
    }
    
    @Override
    public boolean checkSystem(String variant)
    {
        boolean success = true;
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        logNotice("checkSystem (" + variant + ") ------------------");

        try
        {
            boolean allTests = variant.equalsIgnoreCase("all") || variant.equals("");
            if (variant.equals("basic") || allTests)
            {
                logNotice("basic check ------");
                logNotice("  mIntakeMotorRight:\n" + mIntakeMotorRight.dumpState());
                logNotice("  mIntakeMotorLeft:\n" + mIntakeMotorLeft.dumpState());
                logNotice("  isCubeHeld: " + isCubeHeld());
            }
            if (variant.equals("solenoid") || allTests)
            {
                logNotice("solenoid check ------");
                logNotice("grabbing 1s");
                setSolenoidsToGrabbing();
                Timer.delay(1.0);
                logNotice("opening 1s");
                setSolenoidsToOpening();
                Timer.delay(1.0);
                logNotice("floating 1s");
                setSolenoidsToFloating();
                Timer.delay(1.0);
                logNotice("  isCubeHeld: " + isCubeHeld());
                logNotice("slow close 1s");
                mFloatingSolenoid.set(kFloatingSolenoidEngaged);
                mGrabbingSolenoid.set(kGrabbingSolenoidEngaged);
            }
            if (variant.equals("motors") || allTests)
            {
                logNotice("motors check ------");
                logNotice("open arms (2s)");
                Timer.delay(2.0);

                logNotice("left motor fwd .5 (4s)"); // in
                mIntakeMotorLeft.set(.5);
                Timer.delay(4.0);
                logNotice("  current: " + mIntakeMotorLeft.getOutputCurrent());
                mIntakeMotorLeft.set(0);

                logNotice("right motor fwd .5 (4s)");
                mIntakeMotorRight.set(.5);
                Timer.delay(4.0);
                logNotice("  current: " + mIntakeMotorRight.getOutputCurrent());
                mIntakeMotorRight.set(0);

                logNotice("both motors rev .5 (4s)"); // out
                mIntakeMotorLeft.set(-.5);
                mIntakeMotorRight.set(-.5);
                Timer.delay(4.0);
                logNotice("  left current: " + mIntakeMotorLeft.getOutputCurrent());
                logNotice("  right current: " + mIntakeMotorRight.getOutputCurrent());
                mIntakeMotorLeft.set(0);
                mIntakeMotorRight.set(0);

                Timer.delay(.5); // let motors spin down
            }
            if (variant.equals("IRSensor") || allTests)
            {
                logNotice("SensorCheck");
                logNotice("Is Cube Held?");
                logNotice("Cube Distance: " + mCubeHeldSensor.getDistance());
            }
        }
        catch (Throwable e)
        {
            success = false;
            logException("checkSystem", e);
        }

        logNotice("--- finished ---------------------------");
        return success;
    }
}
