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

    private static ArticulatedGrabber sInstance = null;
    private TalonSRX4915 mPositionMotor = null;
    private LazySolenoid mGrabber = null;
    private LazySolenoid mGrabberSetup = null;
    private AnalogInput mPotentiometer = null;
    private DigitalInput mLimitSwitch = null;

    //command for other subsystems to start the subsystem
    public static ArticulatedGrabber getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new ArticulatedGrabber();
        }
        return sInstance;
    }

    //SystemState corresponds to the two values being tracked
    public class SystemState
    {

        //Indicates the position of the flipper arm
        public int articulatorPosition;
        //two values- open or closed
        public boolean grabberOpen;
        public boolean grabberSetup;
    }

    public enum WantedState
    {
        //Corresponding Variables    //Interpretation of each wanted state
        //For WantedState            //In terms of what is actually happening

        TRANSPORT, //position: 0, open: false   //the cube is grabbed and flat against the lift
        PREPARE_DROP, //position: 1, open: false   //the cube is grabbed and held over the switch/scale
        GRAB_CUBE, //position: 2, open: false   //the cube is grabbed and on the ground
        PREPARE_EXCHANGE, //position: 0, open: true    //the cube is not grabbed and flat on the lift
        RELEASE_CUBE, //position: 1, open: true    //the cube is just released over the switch/scale
        PREPARE_INTAKE //position: 2, open: true    //the claw is over the cube on the ground but not grabbed
    }

    //Implement the positions 
    //TODO: once testing begins add default positions for the potentiometer
    private int scalePosition = 2;
    private int intakePosition = 1;
    private int homePosition = 0;
    private int acceptablePositionError = 0;
    private int potValue;

    //Maximum Motor Speed: used in the handlePosition method and the config for mPositionMotor
    private double maxMotorSpeed = 1.0;

    //States are created
    private SystemState mNextState = new SystemState();
    private SystemState mSystemState = new SystemState();//used for analyzing SystemState
    private WantedState mWantedState = WantedState.PREPARE_EXCHANGE; //???

    private ArticulatedGrabber()//initializes subsystem
    {
        boolean success = true;
        
        //add ports
        try {
            mPositionMotor = TalonSRX4915Factory.createDefaultMotor(Constants.kGrabberFlipperMotorId);
            mPositionMotor.configOutputPower(true, .5, 0, 0.1, 0, -0.1);//may be negative in last number
            mGrabber = new LazySolenoid(Constants.kGrabberSolenoidId);
            mGrabberSetup = new LazySolenoid(Constants.kGrabberSetupSolenoidId);
            mPotentiometer = new AnalogInput(Constants.kGrabberAnglePotentiometerId);
            mLimitSwitch = new DigitalInput(Constants.kFlipperHomeLimitSwitchId);
            
            if (mGrabber.isValid())
            {
                success = false;
                logWarning("Grabber1 Invalid");
            }
            if (mGrabberSetup.isValid())
            {
                success = false;
                logWarning("Grabber Setup Invalid");
            }
            if (!mPositionMotor.isValid())
            {
                success = false;
                logWarning("PositionMotor Invalid");
            }
        } catch (Exception e) {
            logError("Failed to instantiate hardware objects.");
            Logger.logThrowableCrash(e);
            success = false;
        }
        logInitialized(success);
    }

    //main loop in which everything is called especially the handling method
    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)//calibration
        {
            synchronized (ArticulatedGrabber.this)
            {
                mSystemState.articulatorPosition = mPotentiometer.getAverageValue();
                mSystemState.grabberOpen = mGrabber.get();
                mSystemState.grabberSetup = mGrabberSetup.get();
                //This Method runs at the beginning of auto and teleop
                //mGrabber1.set(false);
                //mGrabber2.set(true);
                //mSystemState.articulatorPosition = 0;
                //mSystemState.grabberOpen = false;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ArticulatedGrabber.this)
            {
                if (!mSystemState.grabberSetup)
                {
                    mGrabberSetup.set(true);
                    mSystemState.grabberSetup = true;
                }

                potValue = mPotentiometer.getAverageValue();
                //Handle calls
                mNextState.articulatorPosition = handleGrabberPosition(potValue);
                mNextState.grabberOpen = handleGrabberState(potValue);

                //Log change in state/position then assigns current state
                if (mNextState.grabberOpen != mSystemState.grabberOpen)
                {
                    dashboardPutString("State change: ", "Articulated Grabber state from "
                            + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                    logInfo("State change: Articulated Grabber state from "
                            + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                }
                if (mNextState.articulatorPosition != mSystemState.articulatorPosition)
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
        public void onStop(double timestamp)//stop
        {
            synchronized (ArticulatedGrabber.this)
            {
                stop();
            }
        }

    };

    //Looks at the current state of the grabber mechanism
    //compares it to the desired state the prescribes action
    private boolean handleGrabberState(int potValue)
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
                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))
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
        //Dashboard Logging
        dashboardPutState("position: " + mSystemState.articulatorPosition + " grabber: "
                + mSystemState.grabberOpen);
        dashboardPutNumber("potentiometer value: ", mPotentiometer.getAverageValue());
        dashboardPutBoolean("limit switch pressed: ", !mLimitSwitch.get());
        dashboardPutNumber("position motor", mPositionMotor.getOutputCurrent());
    }

    @Override
    public synchronized void stop()//stop function
    {
        mPositionMotor.set(0.0);
        mGrabber.set(false);
        mGrabberSetup.set(false);
        mSystemState.grabberSetup = false;
        mSystemState.grabberOpen = false;
    }

    @Override
    public void zeroSensors()
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
    
    public boolean checkSystem()
    {
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }else {
            logNotice("mArticulatedGrabber: checkSystem() ---------------------------------");
            
            logNotice("mPositionMotor:\n" + mPositionMotor.dumpState());
            
            logNotice("WantedState: RELEASE_CUBE");
            this.setWantedState(ArticulatedGrabber.WantedState.RELEASE_CUBE);
            Timer.delay(1.0);
            logNotice("mPositionMotor: " + mPositionMotor.getOutputCurrent());
            Timer.delay(2.0);
            
            logNotice("WantedState: GRAB_CUBE");
            this.setWantedState(ArticulatedGrabber.WantedState.GRAB_CUBE);
            Timer.delay(1.0);
            logNotice("mPositionMotor: " + mPositionMotor.getOutputCurrent());
            Timer.delay(2.0);
            
            logNotice("WantedState: PREPARE_EXCHANGE");
            this.setWantedState(ArticulatedGrabber.WantedState.PREPARE_EXCHANGE);
            Timer.delay(1.0);
            logNotice("mPositionMotor: " + mPositionMotor.getOutputCurrent());
            Timer.delay(2.0);
            
            logNotice("WantedState: RELEASE_CUBE");
            this.setWantedState(ArticulatedGrabber.WantedState.RELEASE_CUBE);
            Timer.delay(1.0);
            logNotice("mPositionMotor: " + mPositionMotor.getOutputCurrent());
            Timer.delay(2.0);
            
            logNotice("WantedState: PREPARE_INTAKE");
            this.setWantedState(ArticulatedGrabber.WantedState.PREPARE_INTAKE);
            Timer.delay(1.0);
            logNotice("mPositionMotor: " + mPositionMotor.getOutputCurrent());
            Timer.delay(2.0);
            
            logNotice("WantedState: TRANSPORT");
            this.setWantedState(ArticulatedGrabber.WantedState.TRANSPORT);
            Timer.delay(1.0);
            logNotice("mPositionMotor: " + mPositionMotor.getOutputCurrent());
            Timer.delay(2.0);
            
            logNotice("mArticulatedGrabber: finished ---------------------------------");
            return true;
        }
    }
}