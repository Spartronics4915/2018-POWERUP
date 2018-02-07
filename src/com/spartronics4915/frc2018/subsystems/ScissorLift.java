package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.frc2018.subsystems.LED.SystemState;
import com.spartronics4915.lib.util.Util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/*
 * Notes on solenoid controls (from Riyadth)
 * 
 * To clarify the brake solenoid: As the scissor lift rises to the point where the 
 * software wants to stop it, the software should first engage the brake solenoid, 
 * which will hold the scissor lift in place. The software may want to continue to 
 * allow air in the lift solenoid for a period of time afterwards, so that the 
 * scissor lift is pressurized firmly. If we turn off the lift solenoid too soon 
 * there is a chance that the lift will be "bouncy", and could go down a bit while 
 * positioning. The brake only keeps the scissor from rising higher, and does not 
 * prevent it from going lower.
 * After applying the brake, in order to move the scissor lift again, the brake must 
 * be released, and then the scissor lift must be lowered a small amount, even if 
 * the desire is to raise it up higher. This is because there is a mechanical cam 
 * that is under tension, holding the lift from going higher, until the lift is 
 * lowered slightly. Of course, if the lift needs to be lowered anyway, it will be 
 * free to do so after the brake is released (ie, no small motion required first).
 */

/**
 * The scissor lift is controlled by pnuematics. It has multiple set positions
 * and variable height.
 * The key thing here is that its system state is the highly variable position
 * of the lifter.
 * This is why the state machine looks different.
 */
public class ScissorLift extends Subsystem
{

    // we're a singleton
    private static ScissorLift sInstance = null;

    public static ScissorLift getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new ScissorLift();
        }
        return sInstance;
    }

    // Default values for potentiometer target positions
    // Since we anticipate "drift", we obtain actual values 
    // from dashboard during zeroSensors.  That said, we lose those
    // values during reboot, so we must update these compile-time constants
    // with our best-known values.
    private static final int kDefaultLowValue = 0;
    private static final int kDefaultMidValue = 1000;
    private static final int kDefaultHighValue = 2040;
    private static final int kPotentiometerAllowedError = 32;
    private static final double kBrakeTimePeriod = .1;
    private static final double kUnbrakeTimePeriod = .1;

    public enum SystemState
    {
        OFF, // == 0
        RAISING,
        LOWERING,
        HOLDING,
        BRAKING,
        UNBRAKING,
    }

    public enum WantedState
    {
        OFF, // == 0,  is off different from low?
        LOW,
        MIDDLE,
        HIGH,
        // might add HOOK/CLIMB
    }

    private SystemState mSystemState = SystemState.OFF;
    private WantedState mWantedState = WantedState.LOW;
    private int[] mWantedStateMap = new int[4]; // set in zeroSensors()
    private AnalogInput mPotentiometer;
    private int mMeasuredValue; // [0,4095]
    private Solenoid mRaiseSolenoid;
    private Solenoid mLowerSolenoid;
    private Solenoid mHoldSolenoid;
    private Timer mTimer;

    // Actuators and sensors should be initialized as private members with a value of null here

    private ScissorLift()
    {
        boolean success = true;

        mPotentiometer = new AnalogInput(Constants.kScissorPotentiometer);
        mRaiseSolenoid = new Solenoid(Constants.kScissorUpSolenoidId);
        mLowerSolenoid = new Solenoid(Constants.kScissorDownSolenoidId);
        mHoldSolenoid = new Solenoid(Constants.kScissorBrakeSolenoidId);

        success = Util.validateSolenoid(mRaiseSolenoid) &&
                Util.validateSolenoid(mLowerSolenoid) &&
                Util.validateSolenoid(mHoldSolenoid);

        // TODO: check potentiometer value to see if its connected.
        //  this would be valid if we can count on the lift being
        //  in a reasonable state (ie lowered).  We can't detect
        //  wiring mishaps, since reading the analog pin will always
        //  return a value.

        logInitialized(success);
    }

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                mSystemState = SystemState.OFF;
                zeroSensors(); // make sure mWantedStateMap is initialized
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                SystemState newState = updateState();
                if (newState != mSystemState)
                {
                    dashboardPutState(mSystemState.toString());
                    mSystemState = newState;
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

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
        dashboardPutWantedState(wantedState.toString());
    }

    // based on the combination of wanted state, current state and the current potentiometer value,
    // transfer the current system state to another state.
    private SystemState updateState()
    {
        int targetValue = mWantedStateMap[mWantedState.ordinal()];
        SystemState nextState = mSystemState;
        mMeasuredValue = mPotentiometer.getAverageValue();
        if (Util.epsilonLessThan(mMeasuredValue, targetValue, kPotentiometerAllowedError))
        {
            if (mSystemState != SystemState.RAISING)
            {
                if (mSystemState != SystemState.UNBRAKING)
                {
                    mTimer.reset();
                    mTimer.start();
                    mHoldSolenoid.set(false);
                    nextState = SystemState.UNBRAKING;
                }
                else if (mTimer.hasPeriodPassed(kUnbrakeTimePeriod))
                {
                    mLowerSolenoid.set(false);
                    mRaiseSolenoid.set(true);
                    nextState = SystemState.RAISING;
                }
                // else nextState = SystemState.UNBRAKING // (which was current state);

            }
            // else nextState = SystemState.RAISING // (which was current state);
        }
        else if (Util.epsilonGreaterThan(mMeasuredValue, targetValue, kPotentiometerAllowedError))
        {
            if (mSystemState != SystemState.LOWERING)
            {
                if (mSystemState != SystemState.UNBRAKING)
                {
                    mTimer.reset();
                    mTimer.start();
                    mHoldSolenoid.set(false);
                    nextState = SystemState.UNBRAKING;
                }
                else if (mTimer.hasPeriodPassed(kUnbrakeTimePeriod))
                {
                    mLowerSolenoid.set(true);
                    mRaiseSolenoid.set(false);
                    nextState = SystemState.LOWERING;
                }
                // else nextState = SystemState.UNBRAKING // (which was current state);
            }
            // else nextState = SystemState.LOWERING // (which was current state);
        }
        else
        {
            // we're on target
            if (mSystemState != SystemState.HOLDING)
            {
                if (mSystemState != SystemState.BRAKING)
                {
                    mTimer.reset();
                    mTimer.start();
                    mHoldSolenoid.set(true);
                    nextState = SystemState.BRAKING;
                }
                else if (mTimer.hasPeriodPassed(kBrakeTimePeriod))
                {
                    mLowerSolenoid.set(false);
                    mRaiseSolenoid.set(false);
                    nextState = SystemState.HOLDING;
                }
                // else nextState = SystemState.BRAKING // (which was current state);
            }
            // else nextState = SystemState.HOLDING // (which was current state);
        }
        return nextState;
    }

    @Override
    public void outputToSmartDashboard()
    {
        dashboardPutNumber("Potentiometer", mMeasuredValue);
    }

    @Override
    public synchronized void stop()
    {
        setWantedState(WantedState.OFF);
    }

    @Override
    public void zeroSensors()
    {
        // we update our value map here based on smart dashboard values.
        // we could also auto-calibrate our 'zero" here if we're in a known position.
        mWantedStateMap[WantedState.OFF.ordinal()] =
                dashboardGetNumber("Target1", kDefaultLowValue).intValue();
        mWantedStateMap[WantedState.LOW.ordinal()] =
                dashboardGetNumber("Target1", kDefaultLowValue).intValue();
        mWantedStateMap[WantedState.MIDDLE.ordinal()] =
                dashboardGetNumber("Target2", kDefaultMidValue).intValue();
        mWantedStateMap[WantedState.HIGH.ordinal()] =
                dashboardGetNumber("Target3", kDefaultHighValue).intValue();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }
}
