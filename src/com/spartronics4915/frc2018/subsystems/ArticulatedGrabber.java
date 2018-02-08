package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
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
    //add port numbers
    //change out of null
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
        public double articulatorPosition;//three values- 0==holding, 1==dumping, 2==loading
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
    
    double potentiometerError = 0;
    private final double scalePosition = 0;
    private final double intakePosition = 0;
    private final double acceptablePositionError = 0;
    
    private SystemState mSystemState = new SystemState();//used for analyzing SystemState
    private WantedState mWantedState = WantedState.TRANSPORT; //double check this 

    // Actuators and sensors should be initialized as private members with a value of null here

    private ArticulatedGrabber()//initializes subsystem
    {
        //add ports
        mPositionMotor = TalonSRX4915Factory.createDefaultMotor(5);
        mGrabber1 = new Solenoid(3);
        mGrabber2 = new Solenoid(4);
        mPotentiometer = new AnalogInput(1);
        mLimitSwitch = new DigitalInput(1);
        
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false

        logInitialized(success);
    }

    private Loop mLoop = new Loop() {//main loop in which everything is called  //goes through handling through again and again

        @Override
        public void onStart(double timestamp)//calibration
        {
            mGrabber2.set(true); 
            synchronized(ArticulatedGrabber.this)
            {
                mSystemState.articulatorPosition = 0;
                mSystemState.grabberOpen = false;
            }
        }
        
        @Override
        public void onLoop(double timestamp)
        {
            SystemState newState = new SystemState();//creates newstate

            synchronized(ArticulatedGrabber.this)
            {
               
                newState.articulatorPosition = handleGrabberPosition();
                newState.grabberOpen = handleGrabberState();
                dashboardPutWantedState(""+mWantedState+"");
                dashboardPutState("position: " + mSystemState.articulatorPosition+ "");
                dashboardPutNumber("potentiometer value: ", mPotentiometer.getVoltage());
                dashboardPutBoolean("limit switch pressed: ", !mLimitSwitch.get());
                /* 
                if (mSystemState.grabberOpen == false && mSystemState.articulatorPosition == 0)//holding flat
                    {
                    }
                else if (mSystemState.grabberOpen == false && mSystemState.articulatorPosition == 1)//holding over scale
                    {
                    }
                else if (mSystemState.grabberOpen == false && mSystemState.articulatorPosition == 2)//holding over ground
                    {
                    }
                else if (mSystemState.grabberOpen == true && mSystemState.articulatorPosition == 0)//open flat
                    {
                    }
                else if (mSystemState.grabberOpen == true && mSystemState.articulatorPosition == 1)//open over scale
                    {
                    //make newState equal handle method run handleGrabberPosition and handleGrabberState
                    }
                else if (mSystemState.grabberOpen == true && mSystemState.articulatorPosition == 2)//open over ground
                    {
                    }
                else 
                    {
                     //newState = mSystemState;
                    }*/
                
                /*logNotice("WantedState: " + mWantedState);
                newState.grabberOpen = handleGrabberState();
                newState.articulatorPosition = handleGrabberPosition();
                */
                //Log change in state/position then assigns current state
                if (newState.grabberOpen != mSystemState.grabberOpen) {
                    dashboardPutString("State change: ", "Articulated Grabber state from " + mSystemState.grabberOpen + "to" + newState.grabberOpen);
                    logInfo("Articulated Grabber state from " + mSystemState.grabberOpen + "to" + newState.grabberOpen);
                    mSystemState.grabberOpen = newState.grabberOpen;
                }
                if (newState.articulatorPosition != mSystemState.articulatorPosition) {
                    dashboardPutString("Position change: ", "Articulated Grabber position from " + mSystemState.articulatorPosition + "to" + newState.articulatorPosition);
                    logInfo("Articulated Grabber position from " + mSystemState.articulatorPosition + "to" + newState.articulatorPosition);
                    mSystemState.articulatorPosition = newState.articulatorPosition;
                }
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

    private boolean handleGrabberState()//same thing as positon but with bool
    {
        // You should probably be transferring state and controlling actuators in here
        switch (mWantedState) {
            case TRANSPORT:
                //logNotice("handleGrabberState.TRANSPORT running");

                mGrabber1.set(false);
                return false;
                
            case PREPARE_DROP:
                //logNotice("handleGrabberState.PREPARE_DROP running");
              
                mGrabber1.set(false);
                return false;
                
            case GRAB_CUBE:
                //logNotice("handleGrabberState.GRAB_CUBE running");
                
                mGrabber1.set(false);
                return false;
                
            case PREPARE_EXCHANGE:
                //logNotice("handleGrabberState.PREPARE_EXCHANGE running");
                //ACTION HERE
                mGrabber1.set(true);
                return true;
                
            case RELEASE_CUBE:
                //logNotice("handleGrabberState.RELEASE_CUBE running");
                if ((mSystemState.articulatorPosition > scalePosition - acceptablePositionError) && (mSystemState.articulatorPosition < scalePosition + acceptablePositionError))//???
                {
                    mGrabber1.set(true);
                    return true;
                }
                else
                {
                    return false;
                }
                
            case PREPARE_INTAKE:
                //logNotice("handleGrabberState.PREPARE_INTAKE running");

                mGrabber1.set(true);
                return true;
                
            default:
                mGrabber1.set(true);

                return false;
        }
    }
    
    private double handleGrabberPosition()//code for grabber position   //compares current state to wanted state + prescribes an action to make wanted state equal to current state
    {
        // You should probably be transferring state and controlling actuators in here
        switch (mWantedState) {
            case TRANSPORT:
                //logNotice("handleGrabberPosition.TRANSPORT running");

              //ACTION HERE
                
                //or run until you hit the limit switch
                //where is the reset function
                if (!mLimitSwitch.get())
                {
                    mPositionMotor.set(0);
                    potentiometerError = mPotentiometer.getVoltage();
                    return 0;
                }
                else
                {
                    mPositionMotor.set(-1.0);//change later
                    return mPotentiometer.getVoltage();
                }
            
            case PREPARE_DROP:
                //logNotice("handleGrabberPosition.PREPARE_DROP running");

                if ((mSystemState.articulatorPosition > scalePosition - acceptablePositionError) && (mSystemState.articulatorPosition < scalePosition + acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition > scalePosition)
                {
                    mPositionMotor.set(-1.0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition < scalePosition)
                {
                    mPositionMotor.set(1.0);
                    return mPotentiometer.getVoltage();
                }
                else
                {
                    return mPotentiometer.getVoltage();
                }
                                
            case GRAB_CUBE:
                //logNotice("handleGrabberPosition.GRAB_CUBE running");

                if ((mSystemState.articulatorPosition > intakePosition - acceptablePositionError) && (mSystemState.articulatorPosition < intakePosition + acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition > intakePosition)
                {
                    mPositionMotor.set(-1.0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition < intakePosition)
                {
                    mPositionMotor.set(1.0);
                    return mPotentiometer.getVoltage();
                }
                else
                {
                    return mPotentiometer.getVoltage();
                }
                
            case PREPARE_EXCHANGE:
                //logNotice("handleGrabberPosition.PREPARE_EXCHANGE running");

                if (!mLimitSwitch.get())
                {
                    mPositionMotor.set(0);
                    return mPotentiometer.getVoltage();
                }
                else
                {
                    mPositionMotor.set(-1.0);
                    return mPotentiometer.getVoltage();
                }
                
            case RELEASE_CUBE:
                //logNotice("handleGrabberPosition.RELEASE_CUBE running");

                if ((mSystemState.articulatorPosition > scalePosition - acceptablePositionError) && (mSystemState.articulatorPosition < scalePosition + acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition > scalePosition)
                {
                    mPositionMotor.set(-1.0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition < scalePosition)
                {
                    mPositionMotor.set(1.0);
                    return mPotentiometer.getVoltage();
                }
                else
                {
                    return mPotentiometer.getVoltage();
                }
                
            case PREPARE_INTAKE:
                //logNotice("handleGrabberPosition.PREPARE_INTAKE running");
                if ((mSystemState.articulatorPosition < intakePosition - acceptablePositionError) && (mSystemState.articulatorPosition > intakePosition + acceptablePositionError))
                {
                    mPositionMotor.set(0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition > intakePosition)
                {
                    mPositionMotor.set(-1.0);
                    return mPotentiometer.getVoltage();
                }
                if (mSystemState.articulatorPosition < intakePosition)
                {
                    mPositionMotor.set(1.0);
                    return mPotentiometer.getVoltage();
                }
                else
                {
                    return mPotentiometer.getVoltage();
                }
                
            default:
                //logNotice("handleGrabberPosition.default running");
                return mPotentiometer.getVoltage();

        }
    }
    
    
    public void setWantedState(WantedState wantedState)//will be called when the controller is used
    {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard()//code for outputting logs to the dashboard
    {
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