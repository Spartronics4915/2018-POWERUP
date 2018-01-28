package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;

/**
 * The harvester is a set of two collapsible rollers that pull in and hold
 * a cube. This cube is then in a position where it can be picked up by the
 * articulated grabber.
 */
public class Harvester extends Subsystem
{
    private static Harvester sInstance = null;

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
        FIXMEING,
    }

    public enum WantedState
    {
        FIXME,
    }

    private SystemState mSystemState = SystemState.FIXMEING;
    private WantedState mWantedState = WantedState.FIXME;
    
    // Actuators and sensors should be initialized as private members with a value of null here
    
    private Harvester()
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
            synchronized(Harvester.this)
            {
                mSystemState = SystemState.FIXMEING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(Harvester.this)
            {
                SystemState newState;
                switch (mSystemState) {
                    case FIXMEING:
                        newState = handleFixmeing();
                        break;
                    default:
                        newState = SystemState.FIXMEING;
                }
                if (newState != mSystemState) {
                    logInfo("Harvester state from " + mSystemState + "to" + newState);
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized(Harvester.this)
            {
                stop();
            }
        }
        
    };
    
    private SystemState handleFixmeing() {
        // You should probably be transferring state and controlling actuators in here
        return SystemState.FIXMEING;
    }
    
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
