package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        CLOSING,
        OPENING,
        HARVESTING,
        EJECTING,
        HUGGING,
    }

    public enum WantedState
    {
        CLOSE,
        OPEN,
        HARVEST,
        EJECT,
        HUG,
    }

    private SystemState mSystemState = SystemState.CLOSING;
    private WantedState mWantedState = WantedState.CLOSE;
    
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
                mSystemState = SystemState.CLOSING;

            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(Harvester.this)
            {
                SystemState newState;
                switch (mSystemState) {
                    case CLOSING:
                        newState = handleClosing();
                        break;
                    case OPENING:
                        newState = handleOpening();
                        break;
                    case HARVESTING:
                        newState = handleHarvesting();
                        break;
                    case EJECTING:
                        newState = handleEjecting();
                        break;
                    case HUGGING:
                        newState = handleHugging();
                        break;      
                    default:
                        newState = handleClosing();
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
    
    private SystemState defaultStateTransfer()
    {

        switch (mWantedState)
        {
            case CLOSE:
                return SystemState.CLOSING;
            case OPEN:
                return SystemState.OPENING;
            case HARVEST:
                return SystemState.HARVESTING;
            case EJECT:
                return SystemState.EJECTING;
            case HUG:
                return SystemState.HUGGING;
            default:
                return mSystemState;
        }
    }
    
    private SystemState handleClosing() {
        //motors off and bars in
        
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.OPEN) 
        {
            return defaultStateTransfer(); 
        }
        else
        {
            return SystemState.CLOSING;
        }
            
    }
    
    private SystemState handleOpening() {
        //motors off and bars out
        
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.HARVEST) 
        {
            return defaultStateTransfer(); 
        }
        else
            {
            return SystemState.OPENING;
            }
    }
    
    private SystemState handleHarvesting() {
        //motors on forward and bars closing, hug when cube is gone
        
        // You should probably be transferring state and controlling actuators in here
        if (mWantedState == WantedState.HUG || mWantedState == WantedState.EJECT || mWantedState == WantedState.OPEN) 
        {
            return defaultStateTransfer(); 
        }
        else
        {
            return SystemState.HARVESTING;
        }
    }
    
    private SystemState handleEjecting() {
        //motors in reverse and bars closing, close when cube is gone
        
        // You should probably be transferring state and controlling actuators in here
               if (mWantedState == WantedState.OPEN || mWantedState == WantedState.CLOSE) 
        {
            return defaultStateTransfer(); 
        }
        else
        {
            return SystemState.EJECTING;
        }
    }
   
    private SystemState handleHugging() {
        //motors off and bars closing go to closed when cube is gone
        
        // You should probably be transferring state and controlling actuators in here
                if (mWantedState == WantedState.HARVEST || mWantedState == WantedState.EJECT || mWantedState == WantedState.OPEN) 
        {
            return defaultStateTransfer(); 
        }
        else
        {
            return SystemState.HUGGING;
        }
    }
    
    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }
    
    @Override
    public void outputToSmartDashboard()
    {
        SmartDashboard.putString(this.getName()+"/SystemState", this.mSystemState + "");
        SmartDashboard.putString(this.getName()+"/WantedState", this.mWantedState + "");
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
