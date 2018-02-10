package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private TalonSRX4915 mWinchPrimary = null;
    private TalonSRX4915 mWinchSecondary = null;
    private DoubleSolenoid mStablizierSolenoid = null;
    // Actuators and sensors should be initialized as private members with a value of null here
    
    private Climber()
    {
        boolean success = true;
        mWinchPrimary = TalonSRX4915Factory.createDefaultMotor(Constants.kClimberWinchPrimaryMotorId);
        mWinchSecondary = TalonSRX4915Factory.createDefaultSlave(Constants.kClimberWinchSecondaryMotorId, Constants.kClimberWinchPrimaryMotorId, false);
        mStablizierSolenoid = new DoubleSolenoid(Constants.kClimberStabilizationSolenoidId1, Constants.kClimberStabilizationSolenoidId2);
        //Set peak power output
        //Set Voltage ramp rate

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
                        break;
                    default:
                        newState = handleIdling();
                        break;
                        
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
       mStablizierSolenoid.set(DoubleSolenoid.Value.kReverse);
   }
    
   /*private SystemState handleClimbingWithFriends()
    {
     
    } */
   
   private SystemState handleClimbing()
    {
       mWinchPrimary.set(1.0);
        if (mWantedState == WantedState.IDLE) 
        {
            mStablizierSolenoid.set(DoubleSolenoid.Value.kReverse);
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
        if (mWantedState == WantedState.CLIMB) {
            mStablizierSolenoid.set(DoubleSolenoid.Value.kForward);
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
        if (mWantedState == WantedState.CLIMB) 
        {
            return SystemState.CLIMBING;
        }
        else 
        {
            return SystemState.HOLDING;
        }
    }

    public void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard()
    {
        SmartDashboard.putString(this.getName() + "/SystemState", this.mSystemState + "");
        SmartDashboard.putString(this.getName() + "/WantedState", this.mWantedState + "");
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
