/* current known bugs, remove this when fixed
 * bug with WantedStates not transitioning ish- should work on actual robot
 * also, if there is time, add an analog control for the articulator
 * set our WantedStates to use 1 button for the grabber, 2-3 for articulator
 */

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

    private static ArticulatedGrabber sInstance = null;  //defining the parts of our subsystem on the robot
    private TalonSRX4915 mPositionMotor = null;
    private LazySolenoid mGrabber = null;
    private LazySolenoid mGrabberSetup = null;
    private AnalogInput mPotentiometer = null;
    private DigitalInput mLimitSwitch = null;

    public static ArticulatedGrabber getInstance()  //returns an instance of articulatedgrabber
    {
        if (sInstance == null)
        {
            sInstance = new ArticulatedGrabber();
        }
        return sInstance;
    }

    public class SystemState  //SystemState corresponds to the two values being tracked
    {
        public int articulatorPosition;  //indicates the position of the flipper arm
        public boolean grabberOpen;      //false == grabber closed, true == grabber open
        public boolean grabberSetup;     //turns on at startup, should always be on
    }

    public enum WantedState  //WantedState should be set when the buttons are pressed
    {
        TRANSPORT,        //grabbing and flat against lift      //position: 0, open: false
        PREPARE_DROP,     //grabbing and over switch/scale      //position: 1, open: false
        GRAB_CUBE,        //grabbing and over the ground        //position: 2, open: false
        PREPARE_EXCHANGE, //not grabbing and flat against lift  //position: 0, open: true
        RELEASE_CUBE,     //not grabbing over the switch/scale  //position: 1, open: true
        PREPARE_INTAKE    //not grabbing over the ground        //position: 2, open: true
        /* public int wantedArticulatorPosition;
         * public boolean wantedGrabberOpen;
         * public boolean wantedGrabberSetup;  //might not need
         */
    }

    //Implement the positions 
    //TODO: once testing begins add default positions for the potentiometer
    private int scalePosition = 0;            //calibration
    private int intakePosition = 0;           //calibration
    private int homePosition = 0;             //calibration
    private int acceptablePositionError = 0;  //margin of error  //TODO set this when we have the robot
    private int potValue;

    private double maxMotorSpeed = 0.3;  //Maximum Motor Speed: used in handlePosition method and config for mPositionMotor

    private SystemState mNextState = new SystemState();
    private SystemState mSystemState = new SystemState();
    private WantedState mWantedState = WantedState.PREPARE_EXCHANGE;
    //private WantedState mWantedState = new WantedState();

    private ArticulatedGrabber()  //sets up everything
    {
        boolean success = true;
        
        try {
            mPositionMotor = TalonSRX4915Factory.createDefaultMotor(Constants.kGrabberFlipperMotorId);
            mPositionMotor.configOutputPower(true, .5, 0, maxMotorSpeed, 0, -maxMotorSpeed);
            mGrabber = new LazySolenoid(Constants.kGrabberSolenoidId);
            mGrabberSetup = new LazySolenoid(Constants.kGrabberSetupSolenoidId);
            mPotentiometer = new AnalogInput(Constants.kGrabberAnglePotentiometerId);
            mLimitSwitch = new DigitalInput(Constants.kFlipperHomeLimitSwitchId);
        } catch (Exception e) {  //catches the failure to contain it to subsystem
            logError("Failed to instantiate hardware objects.");
            Logger.logThrowableCrash(e);
            success = false;
        }

        if (!mGrabber.isValid())  //instantiate your actuator and sensor objects here
        {
            success = false;
            logWarning("Grabber1 Invalid");
        }
        if (!mGrabberSetup.isValid())
        {
            success = false;
            logWarning("Grabber Setup Invalid");
        }
        if (!mPositionMotor.isValid())
        {
            success = false;
            logWarning("PositionMotor Invalid");
        }

        logInitialized(success);
    }

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (ArticulatedGrabber.this)  //runs at the beginning of auto and teleop, sets SystemState accurately
            {
                mSystemState.articulatorPosition = mPotentiometer.getAverageValue();
                mSystemState.grabberOpen = mGrabber.get();
                mSystemState.grabberSetup = mGrabberSetup.get();
                /* mWantedState.wantedArticulatorPosition = mSystemState.articulatorPosition
                 * mWantedState.wantedGrabberOpen = mSystemState.grabberOpen
                 * mWantedState.wantedGrabberSetup = mSystemState.grabberSetup
                 */
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ArticulatedGrabber.this)
            {
                if (!mSystemState.grabberSetup)  //turns on the GrabberSetup solenoid
                {
                    mGrabberSetup.set(true);
                    mSystemState.grabberSetup = true;  //TODO add stagger time to fix the jolt bug
                }
                
                //handles calls
                potValue = mPotentiometer.getAverageValue();  //just cuts down on the number of calls
                mNextState.articulatorPosition = handleGrabberPosition(potValue);
                mNextState.grabberOpen = handleGrabberState(potValue);

                if (mNextState.grabberOpen != mSystemState.grabberOpen)  //logs change in state/position then assigns current state
                {
                    dashboardPutString("State change: ", "Articulated Grabber state from "
                            + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                    logInfo("State change: Articulated Grabber state from "
                            + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                }
                if (mNextState.articulatorPosition != mSystemState.articulatorPosition)   //logs change then updates SystemState
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

    private boolean handleGrabberState(int potValue)  //controls switching states for grabber
    {
        // You should probably be transferring state and controlling actuators in here
        switch (mWantedState)
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
                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))  //TODO test on real robots
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
            /*
             * case mWantedState.wantedGrabberOpen
             *      if (mNextState.grabberOpen)
             *      {
             *          mGrabber.set(false)
             *      }
             *      return false;
             *
             * case !mWantedState.wantedGrabberOpen
             *      if (!mNextState.grabberOpen)
             *      {
             *          mGrabber.set(true)
             *      }
             *      return true;
             *
             * default:
             *      logWarning("Unexpected Case " + mWantedState.toString());
             *      mGrabber.set(false)
             *      return false;
             */
        }
    }

    //code for grabber position compares current state to wanted state
    //then prescribes an action to make wanted state equal to current state
    private int handleGrabberPosition(int potValue)
    {
        //You should probably be transferring state and controlling actuators in here
        switch (mWantedState)
        {
            case TRANSPORT:

                //We may want to check the limit switch when looking at moving to position zero as a safety mechanism
                if (!mLimitSwitch.get())
                {
                    scalePosition += potValue;
                    intakePosition += potValue;
                    homePosition += potValue;
                }
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
                
                if (!mLimitSwitch.get())
                {
                    scalePosition += potValue;
                    intakePosition += potValue;
                    homePosition += potValue;
                }
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
            
            /* case mWantedState.wantedArticulatorPosition
             *      if (Util.epsilonEquals(potValue, mWantedState.wantedArticulatorPosition, acceptablePositionError))
             *      {
             *          mPositionMotor.set(0);
             *          return potValue;
             *      }
             *      else if (potValue < mWantedState.wantedArticulatorPosition)
             *      {
             *          mPositionMotor.set(maxMotorSpeed);
             *          return potValue;
             *      }
             *      else if (potValue > mWantedState.wantedArticulatorPosition)
             *      {
             *          mPositionMotor.set(-maxMotorSpeed);
             *          return potValue;
             *      }
             * default
             *      logWarning("Unexpected Case " + mWantedState.toString());
             *      mPositionMoter.set(0.0)
             *      return potValue;
             * 
             */

        }
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
        dashboardPutWantedState(mWantedState.toString());
    }

    @Override
    public void outputToSmartDashboard()  //dashboard logging
    {
        dashboardPutState("position: " + mSystemState.articulatorPosition + " grabber: "
                + mSystemState.grabberOpen);
        dashboardPutNumber("potentiometer value: ", mPotentiometer.getAverageValue());
        dashboardPutBoolean("limit switch pressed: ", !mLimitSwitch.get());
    }

    @Override
    public synchronized void stop()  //stops
    {
        mPositionMotor.set(0);
        mGrabber.set(false);
        mGrabberSetup.set(false);
        mSystemState.grabberSetup = false;
        mSystemState.grabberOpen = false;  //TODO maybe add something to stop WantedStates from retriggering
    }

    @Override
    public void zeroSensors()  //calibrates sensors by adding the amount of offput from the potentiometer
    {
        if (!mLimitSwitch.get())
        {
            scalePosition += potValue;
            intakePosition += potValue;
            homePosition += potValue;
        }
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }
}
