package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.LazyDoubleSolenoid;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The climber is mostly a winch that pulls some ropes attached to the top of
 * the scissor
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
        DISABLED,
        IDLING,
        PREPARING,
        CLIMBING,
        HOLDING,

    }

    public enum WantedState
    {
        IDLE,
        PREPARE,
        CLIMB,
        HOLD,
    }

    private SystemState mSystemState = SystemState.DISABLED;
    private WantedState mWantedState = WantedState.IDLE;
    private TalonSRX4915 mWinchPrimary = null;
    // Actuators and sensors should be initialized as private members with a value of null here

    private Climber()
    {
        boolean success = true;

        try
        {
            mWinchPrimary =
                    TalonSRX4915Factory.createDefaultMotor(Constants.kClimberWinchPrimaryMotorId);
            mWinchPrimary.configOutputPower(true, 0.5, 0.0, 0.75, 0.0, -0.5);

            if (!mWinchPrimary.isValid())
            {
                logWarning("Primary Winch missing");
                success = false;
            }
        }
        catch (Exception e)
        {
            logError("Couldn't instantiate hardware objects.");
            Logger.logThrowableCrash(e);
        }

        logInitialized(success);
        }
    

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Climber.this)
            {
                mSystemState = SystemState.DISABLED;
                mWantedState = WantedState.IDLE;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Climber.this)
            {
                SystemState newState;
                switch (mSystemState)
                {
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case IDLING:
                        newState = handleIdling();
                        break;
                    case CLIMBING:
                        newState = handleClimbing();
                        break;
                    case DISABLED:
                        newState = handleDisabled();
                        break;
                    case PREPARING:
                        newState = handlePreparing();
                        break;
                    default:
                        newState = handleIdling();
                        break;

                }
                if (newState != mSystemState)
                {
                    logInfo("Climber state from " + mSystemState + "to " + newState);
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (Climber.this)
            {
                stop();
            }
        }

    };

    private SystemState handleClimbing()
    {
        mWinchPrimary.set(1.0);
        if (mWantedState == WantedState.HOLD)
        {
            return SystemState.HOLDING;
        }
        else
        {
            return SystemState.CLIMBING;
        }

    }

    private SystemState handleIdling()
    {
        mWinchPrimary.set(0.0);
        if (mWantedState == WantedState.PREPARE)
        {
            return SystemState.PREPARING;
        }
        else
        {

            return SystemState.IDLING;
        }
    }

    private SystemState handleHolding()
    {
        mWinchPrimary.set(0.0);
        if (mWantedState == WantedState.CLIMB)
        {
            return SystemState.CLIMBING;
        }
        else if (mWantedState == WantedState.IDLE)
        {
            return SystemState.IDLING;
        }
        else
        {
            return SystemState.HOLDING;
        }
    }

    private SystemState handleDisabled()
    {
        mWinchPrimary.set(0.0);
        if (mWantedState == WantedState.IDLE)
        {
            return SystemState.IDLING;
        }
        logWarning("Wanted State is not Idle at init.");
        return SystemState.IDLING;
    }

    private SystemState handlePreparing()
    {
        mWinchPrimary.set(0.0);
        if (mWantedState == WantedState.CLIMB)
        {
            return SystemState.CLIMBING;
        }
        else if (mWantedState == WantedState.IDLE)
        {
            return SystemState.IDLING;
        }
        else
        {
            return SystemState.PREPARING;
        }
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard()
    {
        //TODO: Put climber current on dashboard
        dashboardPutState(this.mSystemState.toString());
        dashboardPutWantedState(this.mWantedState.toString());
        dashboardPutNumber("Current", mWinchPrimary.getOutputCurrent());
    }

    @Override
    public synchronized void stop()
    {
        mWinchPrimary.set(0.0);
        setWantedState(WantedState.IDLE);
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


    @Override
    public boolean checkSystem(String variant)
    {
        boolean success = true;
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        logNotice("checkSystem (" + variant + ") ------------------");

        try
        {
            boolean allTests = variant.equalsIgnoreCase("all") || variant.equals("");
            if (variant.equals("basic") || allTests)
            {
                logNotice("basic check ------");
                logNotice("  mWinchPrimary:\n" + mWinchPrimary.dumpState());
            }
            if (variant.equals("motors") || allTests)
            {
                logNotice("motor check ------");
                logNotice("  fwd .5 4s");
                mWinchPrimary.set(.5); // clockwise, viewed from robot right side.
                Timer.delay(4);
                logNotice("  master current: " + mWinchPrimary.getOutputCurrent());
              
                logNotice("  rev .5 4s");
                mWinchPrimary.set(-.5);
                Timer.delay(4);
                logNotice("  master current: " + mWinchPrimary.getOutputCurrent());
             
                logNotice("  stop");
                mWinchPrimary.set(0);
                
                // XXX: should we run motors individually?  (disable, then reenable follower)
            }
       }
        catch (Throwable e)
        {
            success = false;
            logException("checkSystem", e);
        }

        logNotice("--- finished ---------------------------");
        return success;
    }
}
