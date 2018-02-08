package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;

import edu.wpi.first.wpilibj.DoubleSolenoid;

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
        CLIMBING,
        HOLDING,
        IDLING
    }

    public enum WantedState
    {
        CLIMB,
        HOLD,
        IDLE
    }

    private SystemState mSystemState = SystemState.IDLING;
    private WantedState mWantedState = WantedState.IDLE;
    private TalonSRX4915 mWinchPrimary = TalonSRX4915Factory.createDefaultMotor(Constants.kClimberWinchPrimaryMotorId);
    private TalonSRX4915 mWinchSecondary = TalonSRX4915Factory.createDefaultSlave(Constants.kClimberWinchSecondaryMotorId, Constants.kClimberWinchPrimaryMotorId, false);
    private DoubleSolenoid mStablizerSoleniod = new DoubleSolenoid(Constants.kClimberStabilizationSolenoidId1, Constants.kClimberStabilizationSolenoidId2);
    
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
               setSolenoidReverse();
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized(Climber.this)
            {
                SystemState newState;
                switch (mSystemState) {
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case IDLING:
                        newState = handleIdling();
                        break;
                    case CLIMBING:
                        newState = handleClimbing();
                    default:
                        newState = SystemState.HOLDING;
                        
                }
                if (newState != mSystemState) {
                    logInfo("Climber state from " + mSystemState + "to " + newState);
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
    
   private void setSolenoidReverse()
   {
       mStablizerSoleniod.set(DoubleSolenoid.Value.kReverse);
   }
    
    private SystemState handleClimbing()
    {
        mWinchPrimary.set(1.0);
        mWinchSecondary.set(1.0); //TODO Implement Smart Dashboard
        if (mWantedState == WantedState.IDLE) 
        {
            mStablizerSoleniod.set(DoubleSolenoid.Value.kReverse);   
            return SystemState.IDLING;
        }
        else if (mWantedState == WantedState.HOLD)
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
        mWinchSecondary.set(0.0);
        if (mWantedState == WantedState.CLIMB) {
            mStablizerSoleniod.set(DoubleSolenoid.Value.kForward);
            return SystemState.CLIMBING;
        }
        else 
        {
            return SystemState.IDLING;   
        }
    }

    private SystemState handleHolding()
    {
        mWinchPrimary.set(0.0);
        mWinchSecondary.set(0.0);
        if (mWantedState == WantedState.CLIMB) 
        {
            
            return SystemState.CLIMBING;
        }
            return SystemState.HOLDING;
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
