package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.frc2018.subsystems.LED.SystemState;
import com.spartronics4915.lib.util.Util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;

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

    public enum SystemState
    {
        OFF,
        RAISING,
        LOWERING,
        HOLDING
    }

    public enum WantedState
    {
        OFF, // is off different from low?
        LOW,
        MIDDLE,
        HIGH,
    }

    private SystemState mSystemState = SystemState.OFF;
    private WantedState mWantedState = WantedState.LOW;
    private int[] mWantedStateMap = new int[4]; // set in zeroSensors()
    private AnalogInput mPotentiometer;
    private int mPotValue; // [0,4095]
    private Solenoid mRaiseSolenoid;
    private Solenoid mLowerSolenoid;
    private Solenoid mHoldSolenoid;

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

        // TODO: check potentiomenter value to see if its connected.
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
        mPotValue = mPotentiometer.getAverageValue();
        int targetValue = mWantedStateMap[mWantedState.ordinal()];
        if (Util.epsilonLessThan(mPotValue, targetValue, kPotentiometerAllowedError))
        {
            if (mHoldSolenoid.get())
                mHoldSolenoid.set(false);;
            if (mLowerSolenoid.get())
                mLowerSolenoid.set(false);
            if (!mRaiseSolenoid.get())
                mRaiseSolenoid.set(true);
            return SystemState.RAISING;
        }
        else if (Util.epsilonGreaterThan(mPotValue, targetValue, kPotentiometerAllowedError))
        {
            if (mHoldSolenoid.get())
                mHoldSolenoid.set(false);;
             if (!mLowerSolenoid.get())
                mLowerSolenoid.set(true);
            if (mRaiseSolenoid.get())
                mRaiseSolenoid.set(false);
            return SystemState.LOWERING;
        }
        else
        {
            if (!mLowerSolenoid.get())
                mLowerSolenoid.set(true);
            if (!mRaiseSolenoid.get())
                mRaiseSolenoid.set(true);
            if (mHoldSolenoid.get())
                mHoldSolenoid.set(true);;
            return SystemState.HOLDING;
        }
    }

    @Override
    public void outputToSmartDashboard()
    {
        dashboardPutNumber("Potentiometer", mPotValue);
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
        mWantedStateMap[0] = dashboardGetNumber("Target1", kDefaultLowValue).intValue();
        mWantedStateMap[1] = dashboardGetNumber("Target1", kDefaultLowValue).intValue();
        mWantedStateMap[2] = dashboardGetNumber("Target2", kDefaultMidValue).intValue();
        mWantedStateMap[3] = dashboardGetNumber("Target3", kDefaultHighValue).intValue();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }
}
