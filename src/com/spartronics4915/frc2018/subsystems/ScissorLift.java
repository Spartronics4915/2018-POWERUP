package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;

/**
 * The scissor lift is controlled by pnuematics. It has multiple set positions and variable height.
 * The key thing here is that its system state is the highly variable position of the lifter.
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

    public enum SystemState
    {
        LOW(0), MIDDLE(64), HIGH(128); // You should probably change these names and values
        
        int mPosition;
        private SystemState(int pos)
        {
            mPosition = pos;
        }
        
        public void setPosition(int pos)
        {
            mPosition = pos;
        }
    }

    public enum WantedState
    {
        LOW,
        MIDDLE,
        HIGH,
    }

    private SystemState mSystemState = SystemState.LOW;
    private WantedState mWantedState = WantedState.LOW;
    
    // Actuators and sensors should be initialized as private members with a value of null here
    
    private ScissorLift()
    {
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false

        logInitialized(success);
    }
    
    private Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp)
        {
            synchronized(ScissorLift.this)
            {
                mSystemState = SystemState.LOW;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(ScissorLift.this)
            {
                switch (mSystemState)
                {
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized(ScissorLift.this)
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
        enabledLooper.register(mLoop);
    }
}
