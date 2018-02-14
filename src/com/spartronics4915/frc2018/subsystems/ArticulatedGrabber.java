package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.drivers.LazySolenoid;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * The articulated grabber includes both a grabber and a flipper arm that work
 * in tandem to pick up cubes from the harvester and place them on the switch
 * or the scale. Climbing hooks may be attached to this subsystem at a later
 * time.
 * A potentiometer keeps track of the position of the arm and a limit switch
 * allows for calibration on the fly.
 */

public class ArticulatedGrabber extends Subsystem
{

    private static ArticulatedGrabber sInstance = null; //defining the parts of our subsystem on the robot
    private TalonSRX4915 mPositionMotor = null;
    private LazySolenoid mGrabber = null;
    private LazySolenoid mGrabberSetup = null;
    private AnalogInput mPotentiometer = null;
    private DigitalInput mLimitSwitch1 = null;
    private DigitalInput mLimitSwitch2 = null;

    public static ArticulatedGrabber getInstance() //returns an instance of ArticulatedGrabber
    {
        if (sInstance == null)
        {
            sInstance = new ArticulatedGrabber();
        }
        return sInstance;
    }

    public class SystemState //SystemState corresponds to the two values being tracked
    {

        public int articulatorPosition; //indicates the position of the flipper arm
        public boolean grabberOpen; //false == grabber closed, true == grabber open
        public boolean grabberSetup; //turns on at startup, should always be on
    }

    public enum WantedState //WantedState should be set when the buttons are pressed
    {
        TRANSPORT, //grabbing and flat against lift      //position: 0, open: false
        PREPARE_DROP, //grabbing and over switch/scale      //position: 1, open: false
        GRAB_CUBE, //grabbing and over the ground        //position: 2, open: false
        PREPARE_EXCHANGE, //not grabbing and flat against lift  //position: 0, open: true
        RELEASE_CUBE, //not grabbing over the switch/scale  //position: 1, open: true
        PREPARE_INTAKE //not grabbing over the ground        //position: 2, open: true
    }

    //TODO: once testing begins add default positions for the potentiometer
    private int scalePosition = 2; //calibration
    private int intakePosition = 1; //calibration
    private int homePosition = 0; //calibration
    private int acceptablePositionError = 0; //margin of error  //TODO set this when we have the robot
    private int potValue;

    private double maxMotorSpeed = 1.0; //Maximum Motor Speed: used in handlePosition method and config for mPositionMotor

    private SystemState mNextState = new SystemState();
    private SystemState mSystemState = new SystemState();
    private WantedState mWantedState = WantedState.PREPARE_EXCHANGE;

    private ArticulatedGrabber() //sets up everything
    {
        boolean success = true;

        try
        {
            mPositionMotor =
                    TalonSRX4915Factory.createDefaultMotor(Constants.kGrabberFlipperMotorId);
            mPositionMotor.configOutputPower(true, .5, 0, 0.1, 0, -0.1);
            mGrabber = new LazySolenoid(Constants.kGrabberSolenoidId);
            mGrabberSetup = new LazySolenoid(Constants.kGrabberSetupSolenoidId);
            mPotentiometer = new AnalogInput(Constants.kGrabberAnglePotentiometerId);
            mLimitSwitch1 = new DigitalInput(Constants.kFlipperHomeLimitSwitchId);
            mLimitSwitch2 = new DigitalInput(Constants.kFlipperHome2LimitSwitchId);

            if (!mGrabber.isValid()) //instantiate your actuator and sensor objects here
            {
                success = false;
                logWarning("GrabberOpen Invalid");
            }
            if (!mGrabberSetup.isValid())
            {
                success = false;
                logWarning("GrabberSetup Invalid");
            }
            if (!mPositionMotor.isValid())
            {
                success = false;
                logWarning("PositionMotor Invalid");
            }
        }
        catch (Exception e)
        { //catches the failure to contain it to subsystem
            logError("Failed to instantiate hardware objects.");
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
            synchronized (ArticulatedGrabber.this) //runs at the beginning of auto and teleop, sets SystemState accurately
            {
                mSystemState.articulatorPosition = mPotentiometer.getAverageValue();
                mSystemState.grabberOpen = mGrabber.get();
                mSystemState.grabberSetup = mGrabberSetup.get();
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ArticulatedGrabber.this)
            {
                if (!mSystemState.grabberSetup) //turns on the GrabberSetup solenoid
                {
                    mGrabberSetup.set(true);
                    mSystemState.grabberSetup = true; //TODO add stagger time to fix the jolt bug
                }

                //handles calls
                potValue = mPotentiometer.getAverageValue(); //just cuts down on the number of calls
                mNextState.articulatorPosition = handleGrabberPosition(potValue);
                mNextState.grabberOpen = handleGrabberState(potValue);

                if (mNextState.grabberOpen != mSystemState.grabberOpen) //logs change in state/position then assigns current state
                {
                    dashboardPutString("State change: ", "Articulated Grabber state from "
                            + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                    logInfo("State change: Articulated Grabber state from "
                            + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                }
                if (mNextState.articulatorPosition != mSystemState.articulatorPosition) //logs change then updates SystemState
                {
                    dashboardPutString("Position change: ",
                            "Articulated Grabber position from " + mSystemState.articulatorPosition
                                    + "to" + mNextState.articulatorPosition);
                    logInfo("Position change: Articulated Grabber position from "
                            + mSystemState.articulatorPosition + "to"
                            + mNextState.articulatorPosition);
                }
                mSystemState = mNextState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (ArticulatedGrabber.this)
            {
                stop();
            }
        }

    };

    private boolean handleGrabberState(int potValue) //controls switching states for grabber
    {
        switch (mWantedState) //you should probably be transferring state and controlling actuators in here
        {
            case TRANSPORT:
                if (mNextState.grabberOpen)
                {
                    mGrabber.set(false);
                }
                return false;

            case PREPARE_DROP:
                if (mNextState.grabberOpen)
                {
                    mGrabber.set(false);
                }
                return false;

            case GRAB_CUBE:
                if (mNextState.grabberOpen)
                {
                    mGrabber.set(false);
                }
                return false;

            case PREPARE_EXCHANGE:
                if (!mNextState.grabberOpen)
                {
                    mGrabber.set(true);
                }
                return true;

            case RELEASE_CUBE:
                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError)) //TODO test on real robot
                {
                    if (!mNextState.grabberOpen)
                    {
                        mGrabber.set(true);
                    }
                    return true;
                }
                else
                {
                    return false;
                }

            case PREPARE_INTAKE:

                if (!mNextState.grabberOpen)
                {
                    mGrabber.set(true);
                }
                return true;

            default:
                logWarning("Unexpected Case " + mWantedState.toString());
                mGrabber.set(false);
                return false;
        }
    }

    private int handleGrabberPosition(int potValue) //controls switching states for articulator
    {
        switch (mWantedState) //you should probably be transferring state and controlling actuators in here
        {
            //We may want to check the limit switch when looking at moving to position zero as a safety mechanism
            case TRANSPORT:
                if (Util.epsilonEquals(potValue, homePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                else if (potValue > homePosition)
                {
                    mPositionMotor.set(-maxMotorSpeed);
                    return potValue;
                }
                else if (potValue < homePosition)
                {
                    mPositionMotor.set(maxMotorSpeed);
                    return potValue;
                }
                else
                {
                    return potValue;
                }

            case PREPARE_DROP:
                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                else if (potValue > scalePosition)
                {
                    mPositionMotor.set(-maxMotorSpeed);
                    return potValue;
                }
                else if (potValue < scalePosition)
                {
                    mPositionMotor.set(maxMotorSpeed);
                    return potValue;
                }
                else
                {
                    return potValue;
                }

            case GRAB_CUBE:

                if (Util.epsilonEquals(potValue, intakePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                else if (potValue > intakePosition)
                {
                    mPositionMotor.set(-maxMotorSpeed);
                    return potValue;
                }
                else if (potValue < intakePosition)
                {
                    mPositionMotor.set(maxMotorSpeed);
                    return potValue;
                }
                else
                {
                    return potValue;
                }

            case PREPARE_EXCHANGE:

               if (Util.epsilonEquals(potValue, homePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                else if (potValue > homePosition)
                {
                    mPositionMotor.set(-maxMotorSpeed);
                    return potValue;
                }
                else if (potValue < homePosition)
                {
                    mPositionMotor.set(maxMotorSpeed);
                    return potValue;
                }
                else
                {
                    return potValue;
                }

            case RELEASE_CUBE:

                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                else if (potValue > scalePosition)
                {
                    mPositionMotor.set(-maxMotorSpeed);
                    return potValue;
                }
                else if (potValue < scalePosition)
                {
                    mPositionMotor.set(maxMotorSpeed);
                    return potValue;
                }
                else
                {
                    return potValue;
                }

            case PREPARE_INTAKE:

                if (Util.epsilonEquals(potValue, intakePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                else if (potValue > intakePosition)
                {
                    mPositionMotor.set(-maxMotorSpeed);
                    return potValue;
                }
                else if (potValue < intakePosition)
                {
                    mPositionMotor.set(maxMotorSpeed);
                    return potValue;
                }
                else
                {
                    return potValue;
                }

            default:
                logWarning("Unexpected Case " + mWantedState.toString());
                mPositionMotor.set(0.0);
                return potValue;
        }
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
        dashboardPutWantedState(mWantedState.toString());
    }

    @Override
    public void outputToSmartDashboard() //dashboard logging
    {
        dashboardPutState("position: " + mSystemState.articulatorPosition + " grabber: "
                + mSystemState.grabberOpen);
        dashboardPutNumber("potentiometer value: ", mPotentiometer.getAverageValue());
        dashboardPutBoolean("limitswitch1 pressed: ", !mLimitSwitch1.get());
        dashboardPutBoolean("limitswitch2 pressed: ", !mLimitSwitch2.get());
        dashboardPutNumber("position motor", mPositionMotor.getOutputCurrent());
    }

    @Override
    public synchronized void stop() //stops
    {
        mPositionMotor.set(0);
        mGrabber.set(false);
        mGrabberSetup.set(false);
        mSystemState.grabberSetup = false;
        mSystemState.grabberOpen = false; //TODO maybe add something to stop WantedStates from retriggering SystemStates
    }

    @Override
    public void zeroSensors() //calibrates sensors by adding the amount of offput from the potentiometer
    {
        if (!mLimitSwitch1.get())
        {
            // scalePosition += potValue;
            // intakePosition += potValue;
            // homePosition += potValue;
        }
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    public boolean checkSystem(String variant) //noah's stuff
    {
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        else
        {
            boolean success = true;
            logNotice("checkSystem (" + variant + ") ------------------");
            try
            {
                boolean allTests = variant.equalsIgnoreCase("all") || variant.equals("");
                if (variant.equals("basic") || allTests)
                {
                    logNotice("basic check ------");
                    logNotice("    mPositionMotor:\n" + mPositionMotor.dumpState());
                    logNotice("    mGrabber: " + mGrabber.get());
                    logNotice("    mGrabberSetup: " + mGrabberSetup.get());
                    logNotice("    mPotentiometer: " + mPotentiometer.getValue());
                    logNotice("    mLimitSwitch1: " + mLimitSwitch1.get());
                    logNotice("    mLimitSwitch2: " + mLimitSwitch2.get());
               }

                if (variant.equals("grabber") || allTests)
                {
                    logNotice("grabber check ------");
                    logNotice("  mGrabberSetup on (2s)");
                    mGrabberSetup.set(true);
                    Timer.delay(2.0);
                    logNotice("    pot: " + mPotentiometer.getValue());
                    logNotice("  mGrabber on (2s)");
                    mGrabber.set(true);
                    Timer.delay(2.0);
                    logNotice("    pot: " + mPotentiometer.getValue());
                    logNotice("  both solenoids off");
                    mGrabber.set(false);
                    mGrabberSetup.set(false);
                    Timer.delay(2.0);
                    logNotice("    pot: " + mPotentiometer.getValue());
                }

                if (variant.equals("motor") || allTests)
                {
                    logNotice("motor check ------");
                    logNotice("   fwd .1, 1s");
                    mPositionMotor.set(.1);
                    Timer.delay(1.0);
                    logNotice("    pot: " + mPotentiometer.getValue());
                    logNotice("   rev .1, 1s");
                    mPositionMotor.set(-.1);
                    Timer.delay(1.0);
                    logNotice("    pot: " + mPotentiometer.getValue());
                    mPositionMotor.set(0.0);
                }

                if (variant.equals("motorlimit") || allTests)
                {
                    logNotice("motor check ------");
                    logNotice("   fwd .5 to limit, 5s");
                    Timer t = new Timer();
                    int counter = 0;
                    t.start();
                    mPositionMotor.set(.5);
                    while(true)
                    {
                        if(mLimitSwitch1.get() || mLimitSwitch2.get())
                        {
                            logNotice("limit switch encounterd at " + mPotentiometer.getValue());
                            break;
                        }
                        else
                        if(t.hasPeriodPassed(5))
                        {
                            logError("fwd 5s didn't encounter limit switch!!!!!!!");
                            success = false;
                            break;
                        }
                        else
                        {
                            Timer.delay(.1);
                            if(counter++ % 10 == 0)
                                logNotice("    pot: " + mPotentiometer.getValue());
                        }
                    }
                    mPositionMotor.set(0);
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
}
