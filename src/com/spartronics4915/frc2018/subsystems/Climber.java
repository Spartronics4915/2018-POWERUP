package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.LazyDoubleSolenoid;
import com.spartronics4915.lib.util.Util;
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

    private static final DoubleSolenoid.Value kStabilizing = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value kTuckedAway = DoubleSolenoid.Value.kReverse;

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
    private TalonSRX4915 mWinchSecondary = null;
    private LazyDoubleSolenoid mStabilizerSolenoid = null;
    // Actuators and sensors should be initialized as private members with a value of null here

    private Climber()
    {
        boolean success = true;
        mWinchPrimary =
                TalonSRX4915Factory.createDefaultMotor(Constants.kClimberWinchPrimaryMotorId);
        mWinchSecondary =
                TalonSRX4915Factory.createDefaultSlave(Constants.kClimberWinchSecondaryMotorId,
                        Constants.kClimberWinchPrimaryMotorId, false);
        mStabilizerSolenoid = new LazyDoubleSolenoid(Constants.kClimberStabilizationSolenoidId1,
                Constants.kClimberStabilizationSolenoidId2);
        mWinchPrimary.configOutputPower(true, 0.5, 0.0, 0.75, 0.0, -0.5);
        mWinchSecondary.configOutputPower(true, 0.5, 0.0, 0.75, 0.0, -0.5);

        if (!mWinchPrimary.isValid())
        {
            logWarning("Primary Winch missing");
            success = false;
        }
        if (!mWinchSecondary.isValid())
        {
            logWarning("Secondary Winch missing");
            success = false;
        }
        if (mStabilizerSolenoid.isValid())
        {
            logWarning("Stablizer Solenoid is missing");
            success = false;
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
            mStabilizerSolenoid.set(kStabilizing);
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
        mStabilizerSolenoid.set(kTuckedAway);
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
            mStabilizerSolenoid.set(kTuckedAway);
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

    public boolean checkSystem()
    {
        if (!isInitialized())
        {
            logWarning("can't check un-intialized system");
            return false;
        }
        else
        {
            logNotice("Climber check -----------------------------");

            logNotice("WantedState = IDLE");
            this.setWantedState(Climber.WantedState.IDLE);
            Timer.delay(2.5);

            logNotice("WantedState = PREPARE");
            this.setWantedState(Climber.WantedState.PREPARE);
            Timer.delay(2.5);

            logNotice("WantedState = CLIMB");
            this.setWantedState(Climber.WantedState.CLIMB);
            Timer.delay(2.5);

            logNotice("WantedState = HOLD");
            this.setWantedState(Climber.WantedState.HOLD);
            Timer.delay(2.5);

            this.setWantedState(Climber.WantedState.IDLE);

            return true;

        }
    }
}
