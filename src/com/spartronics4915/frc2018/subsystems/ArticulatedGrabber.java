package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The articulated grabber includes both a grabber and a flipper arm that work
 * in tandem to pick up cubes from the harvester and place them on the switch
 * or the scale. Climbing hooks may be attached to this subsystem at a later
 * time.
 */

public class ArticulatedGrabber extends Subsystem//the class
{

    private static ArticulatedGrabber sInstance = null;//creates a variable for the next function
    private TalonSRX4915 mPositionMotor = null;
    private Solenoid mGrabber1 = null;
    private Solenoid mGrabber2 = null;
    private AnalogInput mPotentiometer = null;
    private DigitalInput mLimitSwitch = null;
    
        
    public static ArticulatedGrabber getInstance()//command for other subsystems to start the subsystem
    {
        if (sInstance == null)
        {
            sInstance = new ArticulatedGrabber();
        }
        return sInstance;
    }

    public class SystemState//the two system states. corresponds to WantedStates
    {
        public double articulatorPosition;
        public boolean grabberOpen;//two values- open or closed
    }

    public enum WantedState
    {
        TRANSPORT, //position: 0, open: false       //the cube is grabbed and flat against the lift
        PREPARE_DROP, //position: 1, open: false    //the cube is grabbed and held over the switch/scale
        GRAB_CUBE, //position: 2, open: false       //the cube is grabbed and on the ground
        PREPARE_EXCHANGE, //position: 0, open: true //the cube is not grabbed and flat on the lift
        RELEASE_CUBE, //position: 1, open: true     //the cube is just released over the switch/scale
        PREPARE_INTAKE //position: 2, open: true    //the claw is over the cube on the ground but not grabbed
    }
    
    //Implement the positions 
    private int scalePosition = 0;
    private int intakePosition = 0;
    private int homePosition = 0;
    private double acceptablePositionError = 0.5;
    
    
    private SystemState mNextState = new SystemState();
    private SystemState mSystemState = new SystemState();//used for analyzing SystemState
    private WantedState mWantedState = WantedState.TRANSPORT; //double check this 

    // Actuators and sensors should be initialized as private members with a value of null here

    private ArticulatedGrabber()//initializes subsystem
    {
        //add ports
        mPositionMotor = TalonSRX4915Factory.createDefaultMotor(Constants.kGrabberFlipperMotorId);
        mPositionMotor.configOutputPower(true, .5, 0, .3, 0, .3);//may be negative in last number
        mGrabber1 = new Solenoid(Constants.kGrabberSolenoidId);
        mGrabber2 = new Solenoid(Constants.kGrabberSetupSolenoidId);
        mPotentiometer = new AnalogInput(Constants.kGrabberAnglePotentiometerId);
        mLimitSwitch = new DigitalInput(Constants.kFlipperHomeLimitSwitchId);
        
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false
        if(!Util.validateSolenoid(mGrabber1)) 
        {
            success = false;
            logWarning("Grabber1 Invalid");
        }
        if(!Util.validateSolenoid(mGrabber2)) 
        {
            success = false;
            logWarning("Grabber2 Invalid");
        }
        if(!mPositionMotor.isValid()) 
        {
            success = false;
            logWarning("PositionMotor Invalid");
        }
        
        logInitialized(success);
    }

    private Loop mLoop = new Loop() {//main loop in which everything is called  //goes through handling through again and again

        @Override
        public void onStart(double timestamp)//calibration
        { 
            synchronized(ArticulatedGrabber.this)
            {
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
            synchronized(ArticulatedGrabber.this)
            {
                //Handle calls
                mNextState.articulatorPosition = handleGrabberPosition(mPotentiometer.getAverageValue());
                mNextState.grabberOpen = handleGrabberState(mPotentiometer.getAverageValue());
                
                //Log change in state/position then assigns current state
                if (mNextState.grabberOpen != mSystemState.grabberOpen) {
                    dashboardPutString("State change: ", "Articulated Grabber state from " + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                    logInfo("Articulated Grabber state from " + mSystemState.grabberOpen + "to" + mNextState.grabberOpen);
                    mSystemState.grabberOpen = mNextState.grabberOpen;
                }
                if (mNextState.articulatorPosition != mSystemState.articulatorPosition) {
                    dashboardPutString("Position change: ", "Articulated Grabber position from " + mSystemState.articulatorPosition + "to" + mNextState.articulatorPosition);
                    logInfo("Articulated Grabber position from " + mSystemState.articulatorPosition + "to" + mNextState.articulatorPosition);
                    mSystemState.articulatorPosition = mNextState.articulatorPosition;
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

    private boolean handleGrabberState(int potValue)//same thing as positon but with bool
    {
        // You should probably be transferring state and controlling actuators in here
        switch (mWantedState) {
            case TRANSPORT:
                //logNotice("handleGrabberState.TRANSPORT running");
                if(mSystemState.grabberOpen) 
                {
                    mGrabber1.set(false);
                }
                return false;
                
            case PREPARE_DROP:
                //logNotice("handleGrabberState.PREPARE_DROP running");
                if(mSystemState.grabberOpen) 
                {
                    mGrabber1.set(false);
                }
                return false;
                
            case GRAB_CUBE:
                //logNotice("handleGrabberState.GRAB_CUBE running");
                if(mSystemState.grabberOpen) 
                {
                    mGrabber1.set(false);
                }
                return false;
                
            case PREPARE_EXCHANGE:
                //logNotice("handleGrabberState.PREPARE_EXCHANGE running");
                if(!mSystemState.grabberOpen) 
                {
                    mGrabber1.set(true);
                }
                return true;
                
            case RELEASE_CUBE:
                //logNotice("handleGrabberState.RELEASE_CUBE running");
                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))
                {
                    if(!mSystemState.grabberOpen) 
                    {
                        mGrabber1.set(true);
                    }
                    return true;
                }
                else
                {
                    return false;
                }
                
            case PREPARE_INTAKE:
                //logNotice("handleGrabberState.PREPARE_INTAKE running");

                if(!mSystemState.grabberOpen) 
                {
                    mGrabber1.set(true);
                }
                return true;
                
            default:
                logWarning("Unexpected Case " + mWantedState.toString());
                mGrabber1.set(false);
                return false;
        }
    }
    
    private double handleGrabberPosition(int potValue)//code for grabber position   //compares current state to wanted state + prescribes an action to make wanted state equal to current state
    {
        // You should probably be transferring state and controlling actuators in here
        switch (mWantedState) {
            case TRANSPORT:
                //logNotice("handleGrabberPosition.TRANSPORT running");

              //ACTION HERE
                
                //or run until you hit the limit switch
                //where is the reset function
                if (Util.epsilonEquals(potValue, homePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                if(!mLimitSwitch.get()) 
                {
                    scalePosition += potValue;
                    intakePosition += potValue;
                    homePosition += potValue;
                }
                if (potValue > homePosition)
                {
                    mPositionMotor.set(-1.0);
                    return potValue;
                }
                if (potValue < homePosition)
                {
                    mPositionMotor.set(1.0);
                    return potValue;
                }
                else
                {
                    return potValue;
                }
            
            case PREPARE_DROP:
                //logNotice("handleGrabberPosition.PREPARE_DROP running");
                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                if (potValue > scalePosition)
                {
                    mPositionMotor.set(-1.0);
                    return potValue;
                }
                if (potValue < scalePosition)
                {
                    mPositionMotor.set(1.0);
                    return potValue;
                }
                else
                {
                    return potValue;
                }
                                
            case GRAB_CUBE:
                //logNotice("handleGrabberPosition.GRAB_CUBE running");

                if (Util.epsilonEquals(potValue, intakePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                if (potValue > intakePosition)
                {
                    mPositionMotor.set(-1.0);
                    return potValue;
                }
                if (potValue < intakePosition)
                {
                    mPositionMotor.set(1.0);
                    return potValue;
                }
                else
                {
                    return potValue;
                }
                
            case PREPARE_EXCHANGE:
                //logNotice("handleGrabberPosition.PREPARE_EXCHANGE running");
                if (Util.epsilonEquals(potValue, homePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                if(!mLimitSwitch.get()) 
                {
                    scalePosition += potValue;
                    intakePosition += potValue;
                    homePosition += potValue;
                }
                if (potValue > homePosition)
                {
                    mPositionMotor.set(-1.0);
                    return potValue;
                }
                if (potValue < homePosition)
                {
                    mPositionMotor.set(1.0);
                    return potValue;
                }
                else
                {
                    return potValue;
                }
                
            case RELEASE_CUBE:
                //logNotice("handleGrabberPosition.RELEASE_CUBE running");

                if (Util.epsilonEquals(potValue, scalePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                if (potValue > scalePosition)
                {
                    mPositionMotor.set(-1.0);
                    return potValue;
                }
                if (potValue < scalePosition)
                {
                    mPositionMotor.set(1.0);
                    return potValue;
                }
                else
                {
                    return potValue;
                }
                
            case PREPARE_INTAKE:
                //logNotice("handleGrabberPosition.PREPARE_INTAKE running");
                if (Util.epsilonEquals(potValue, intakePosition, acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return potValue;
                }
                if (potValue > intakePosition)
                {
                    mPositionMotor.set(-1.0);
                    return potValue;
                }
                if (potValue < intakePosition)
                {
                    mPositionMotor.set(1.0);
                    return potValue;
                }
                else
                {
                    return potValue;
                }
                
            default:
                //logNotice("handleGrabberPosition.default running");
                return potValue;

        }
    }
    
    
    public void setWantedState(WantedState wantedState)//will be called when the controller is used
    {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard()//code for outputting logs to the dashboard
    {
      //Dashboard Logging
        dashboardPutWantedState(""+mWantedState+"");
        dashboardPutState("position: " + mSystemState.articulatorPosition+ "");
        dashboardPutNumber("potentiometer value: ", mPotentiometer.getAverageValue());
        dashboardPutBoolean("limit switch pressed: ", !mLimitSwitch.get());
    }

    @Override
    public synchronized void stop()//stop function
    {
        mPositionMotor.set(0.0);
        //stop motors and servos?
    }

    @Override
    public void zeroSensors()//reset the sensors using the reset thing we had planned i.e. collapse the lift and the motors + set
    {
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }
}