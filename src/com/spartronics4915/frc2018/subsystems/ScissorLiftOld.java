package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.spartronics4915.frc2018.Constants;

/**
 * The scissor lift is controlled by pnuematics. It has multiple set positions
 * and variable height.
 * The key thing here is that its system state is the highly variable position
 * of the lifter.
 * This is why the state machine looks different.
 */
public class ScissorLift extends Subsystem
{

    private static ScissorLift sInstance = null;

    public static ScissorLift getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new ScissorLift();
        }
        return sInstance;
    }

    
    private static final int kPotentiometerAllowedError = 10;
    private Solenoid mScissorLifterSolenoid;
    private Solenoid mScissorLowerSolenoid;
    private Solenoid mScissorBrakeSolenoid;
    
  //potentiometer and associated analog input
    private Potentiometer mPotentiometer;
    private AnalogInput mAnalogInput;

    private double mRetracted = 0;
    private double mSwitch = 60;
    private double mScale = 120;
    private double mClimb = 180;

    private double mSystemState = 0; // Potentiometer Value
    private double mLastSystemState = mSystemState;
    private double mWantedState = mRetracted;
    // Actuators and sensors should be initialized as private members with a value of null here

    private ScissorLift()
    {
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false
        // TODO - Add something to tell if the Solenoids are present
        mScissorLifterSolenoid = new Solenoid(Constants.kScissorUpSolenoidId);
        mScissorLowerSolenoid = new Solenoid(Constants.kScissorDownSolenoidId);
        mScissorBrakeSolenoid = new Solenoid(Constants.kScissorBrakeSolenoidId);
        mRetracted = SmartDashboard.getNumber("ScissorLift/Target1", 0); // Default values are arbitrary
        mSwitch = SmartDashboard.getNumber("ScissorLift/Target2", 60);
        mScale = SmartDashboard.getNumber("ScissorLift/Target3", 120);
        mClimb = SmartDashboard.getNumber("ScissorLift/Target4", 180);
     // initialization for solenoid
        mAnalogInput = new AnalogInput(0);
        mPotentiometer = new AnalogPotentiometer(mAnalogInput, 360, mRetracted);
        logInitialized(success);
    }
    
    // Convert the users logical ideas of these heights into the calibrated values
    
    public enum WantedState
    {
        RETRACTED(0), SWITCH(1), SCALE(2), CLIMB(3);

        int mPosition;

        private WantedState(int pos)
        {
            mPosition = pos;
        }

        public void setPosition(int pos)
        {
            mPosition = pos;  
        }

        public int getPosition()
        {
            return mPosition;
        }
            
    }
    

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                mSystemState = mPotentiometer.get(); 
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                mSystemState = mPotentiometer.get();
                if (mWantedState < (mSystemState - kPotentiometerAllowedError)
                        || mWantedState > (mSystemState + kPotentiometerAllowedError))
                {
                    // increase
                    if (mScissorLowerSolenoid.get())
                    {
                        broadcastState("Starting to Raise");
                        mScissorLowerSolenoid.set(false);
                        mScissorLifterSolenoid.set(true);
                    }
                }
                else if (mWantedState > (mSystemState - kPotentiometerAllowedError)
                        || mWantedState < (mSystemState + kPotentiometerAllowedError))
                {
                    // decrease
                    if (mScissorLifterSolenoid.get())
                    {
                        broadcastState("Starting to Lower");
                        mScissorLifterSolenoid.set(false);
                        mScissorLowerSolenoid.set(true);
                    }
                }
                else if (mLastSystemState != mSystemState)
                {
                    // holding
                    boolean report = false;
                    if (!mScissorLowerSolenoid.get())
                    {
                        mScissorLowerSolenoid.set(true);
                        report = true;
                    }
                    if (!mScissorLifterSolenoid.get())
                    {
                        mScissorLifterSolenoid.set(true);
                        report = true;
                    }
                    if (report)
                        broadcastState("Holding");
                }
                mLastSystemState = mSystemState;

            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                stop();
            }
        }
    };

    public void setWantedState(WantedState wantedState)
    {
        if (wantedState == WantedState.RETRACTED)
        {
            mSystemState = mRetracted;
        }
        else if (wantedState == WantedState.SWITCH)
        {
            mSystemState = mSwitch;
        }
        else if (wantedState == WantedState.SCALE)
        {
            mSystemState = mScale;
        }
        else if (wantedState == WantedState.CLIMB)
        {
            mSystemState = mClimb;
        }
        else
        {
            Logger.error("Trying to set an UNKNOWN STATE");
        }
        
    }

    @Override
    public void outputToSmartDashboard()
    {

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

    // Called to check if we've reached our desired state
    // Eventually we will want it to check for the tolerance; may be a switch statement or multiple methods checking for different goals
    public synchronized boolean isDone()
    {
        {

            if (mSystemState == mWantedState)
                return true;
            return false;
        }
    }
}
