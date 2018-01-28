package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;

/**
 * The climber is mostly a winch that pulls some ropes attached to the top of the scissor
 * lift or the flipper.
 */
public class Climber extends Subsystem
{
    private static Climber sInstance = null;

    public static Climber getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new Climber();
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
    
    private Climber()
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
            synchronized(Climber.this)
            {
                mSystemState = SystemState.FIXMEING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(Climber.this)
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
                    logInfo("Climber state from " + mSystemState + "to" + newState);
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized(Climber.this)
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
