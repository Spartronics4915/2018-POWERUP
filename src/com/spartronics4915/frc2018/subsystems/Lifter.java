package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;

/**
 * The harvester is a set of two collapsible rollers that pull in and hold
 * a cube. This cube is then in a position where it can be picked up by the
 * articulated grabber.
 */
public class Lifter extends Subsystem
{
    private static Lifter sInstance = null;

    public static Lifter getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new Lifter();
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
    
    private Lifter()
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
            synchronized(Lifter.this)
            {
                mSystemState = SystemState.LOW;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(Lifter.this)
            {
                switch (mSystemState)
                {
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized(Lifter.this)
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
