package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;

import edu.wpi.first.wpilibj.Solenoid;
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

    public enum WantedState
    {
        RETRACTED(0), SWITCH(64), SCALE(128);

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

    private static final int kPotentiometerAllowedError = 10;
    private Solenoid mScissorLifterSolenoid;
    private Solenoid mScissorLowerSolenoid;
    private Solenoid mScissorBrakeSolenoid;

    private int mSystemState = 0;
    private WantedState mWantedState = WantedState.RETRACTED;

    // Actuators and sensors should be initialized as private members with a value of null here

    private ScissorLift()
    {
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false
        mScissorLifterSolenoid = new Solenoid(Constants.kScissorUpSolenoidId);
        mScissorLowerSolenoid = new Solenoid(Constants.kScissorDownSolenoidId);
        mScissorBrakeSolenoid = new Solenoid(Constants.kScissorBrakeSolenoidId);
        logInitialized(success);
    }

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                mSystemState = 0;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                if (mWantedState.getPosition() < mSystemState - kPotentiometerAllowedError
                        || mWantedState.getPosition() > mSystemState + kPotentiometerAllowedError)
                {
                    // increase
                    logNotice("Starting to Raise");
                    mScissorLowerSolenoid.set(false);
                    mScissorLifterSolenoid.set(true);
                }
                else if (mWantedState.getPosition() > mSystemState - kPotentiometerAllowedError
                        || mWantedState.getPosition() < mSystemState + kPotentiometerAllowedError)
                {
                    // decrease
                    logNotice("Starting to Lower");
                    mScissorLifterSolenoid.set(false);
                    mScissorLowerSolenoid.set(true);

                }

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
        mWantedState = wantedState;
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
        //  enabledLooper.register(mLoop);
    }

    // Called to check if we've reached our desired state
    // Eventually we will want it to check for the tolerance; may be a switch statement or multiple methods checking for different goals
    public synchronized boolean isDone()
    {
        {

            if (mSystemState == mWantedState.getPosition())
                return true;
            return false;
        }
    }
}
