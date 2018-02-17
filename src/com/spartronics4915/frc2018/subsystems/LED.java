package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

/**
 * The LED subsystem consists of:
 * - DriverLED: for communicating information to drivers (also on dashboard)
 * - VisionLamp: on the front of the robot used for illuminating potential
 * vision targets
 * - potential extensions:
 * - LED light or strip for communicating with human player
 * - LED light or strip to communicate distance to driver
 * 
 * The main things this subsystem has to do is turn each light on, off, or make
 * it blink.
 * The vision lamp is not subject to blinking and is currently not part of the
 * SystemState.
 * 
 * 
 * Since we have independent components, we have multiple wanted states and
 * system states.
 * 
 * @see Subsystem.java
 */
public class LED extends Subsystem
{

    public static final int kDefaultBlinkCount = 4;
    public static final double kDefaultBlinkDuration = 0.2; // seconds for full cycle
    private static final double kDefaultTotalBlinkDuration =
            kDefaultBlinkCount * kDefaultBlinkDuration;

    private static LED mInstance = null;

    public static LED getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new LED();
        }
        return mInstance;
    }

    // Internal state of the DriverLED
    public enum SystemState
    {
        OFF, FIXED_ON, BLINKING, RANGE_FINDING
    }

    public enum WantedState
    {
        OFF, FIXED_ON, BLINK, FIND_RANGE, WARN
    }

    private SystemState mSystemState = SystemState.OFF;
    private WantedState mWantedState = WantedState.OFF;

    private boolean mIsLEDOn, mIsLampOn;
    private Relay mDriverLED;
    private Relay mVisionLamp;
    private boolean mIsBlinking = false;
    private double mBlinkDuration;
    private int mBlinkCount;
    private double mTotalBlinkDuration;

    public LED()
    {
        mDriverLED = new Relay(Constants.kLEDDriverLEDId);
        setDriverLEDOff();

        mVisionLamp = new Relay(Constants.kLEDVisionLampId);
        setVisionLampOff();

        configureBlink(kDefaultBlinkCount, kDefaultBlinkDuration);
 
        logInitialized(true);
    }

    private Loop mLoop = new Loop()
    {

        private double mCurrentStateStartTime;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (LED.this)
            {
                mSystemState = SystemState.OFF;
                mWantedState = WantedState.OFF;
                handleOff();
                mIsBlinking = false;
            }

            mCurrentStateStartTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (LED.this)
            {
                SystemState newState;
                double timeInState = timestamp - mCurrentStateStartTime;
                switch (mSystemState)
                {
                    case OFF:
                        newState = handleOff();
                        break;
                    case FIXED_ON:
                        newState = handleFixedOn();
                        break;
                    case BLINKING:
                        newState = handleBlinking(timeInState);
                        break;
                    case RANGE_FINDING:
                        newState = handleRangeFinding(timeInState);
                        break;
                    default:
                        logError("Fell through on LED states!!");
                        newState = SystemState.OFF;
                }
                if (newState != mSystemState)
                {
                    logInfo("LED state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
               }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            handleOff();
        }
    };

    private SystemState defaultStateTransfer()
    {
        switch (mWantedState)
        {
            case OFF:
                return SystemState.OFF;
            case BLINK:
                return SystemState.BLINKING;
            case WARN:
            case FIND_RANGE:
                return SystemState.RANGE_FINDING;
            case FIXED_ON:
                return SystemState.FIXED_ON;
            default:
                return SystemState.OFF;
        }
    }

    private synchronized SystemState handleOff()
    {
        setDriverLEDOff();
        //setVisionLampOff();
        //We do NOT want the vision lED to be turned off by default
        return defaultStateTransfer();
    }

    private synchronized SystemState handleFixedOn()
    {
        setDriverLEDOn();
        return defaultStateTransfer();
    }

    private synchronized SystemState handleRangeFinding(double timeInState)
    {
        return performBlinking(timeInState);
    }

    private synchronized SystemState handleBlinking(double timeInState)
    {
        return performBlinking(timeInState);
    }

    private SystemState performBlinking(double timeInState)
    {
        if (timeInState > mTotalBlinkDuration)
        {
            setDriverLEDOff();
            // Transition to OFF state and clear wanted state.
            mWantedState = WantedState.OFF;
            return SystemState.OFF;
        }

        int cycleNum = (int) (timeInState / (mBlinkDuration / 2.0));
        if ((cycleNum % 2) == 0)
        {
            setDriverLEDOn();
        }
        else
        {
            setDriverLEDOff();
        }
        return SystemState.BLINKING;
    }

    @Override
    public void outputToSmartDashboard()
    {

    }

    @Override
    public void stop()
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

    // when setWantedState is invoked, we merely trigger a behavior change
    // in looper since it calls defaultStateChange through each of the handlers.
    public synchronized void setWantedState(WantedState state)
    {
        mWantedState = state;
        dashboardPutState(state.toString());
        dashboardPutString("Message", "");
        switch (mWantedState)
        {
            case OFF:
                break;
            case BLINK:
                configureBlink(kDefaultBlinkCount, kDefaultBlinkDuration);
                break;
            case FIND_RANGE:
            case WARN:
                configureBlink(kDefaultBlinkCount*2, .5*kDefaultBlinkDuration);
                break;
            case FIXED_ON:
                break;
            default:
                break;
        }
    }
    
    public synchronized void warnDriver(String msg)
    {
        setWantedState(WantedState.WARN);
        dashboardPutString("Message", msg);
    }

    public synchronized void setDriverLEDOn()
    {
        if (!mIsLEDOn)
        {
            dashboardPutBoolean("DriverLED", true);
            mIsLEDOn = true;
            mDriverLED.set(Relay.Value.kForward);
        }
    }

    public synchronized void setDriverLEDOff()
    {
        if (mIsLEDOn)
        {
            dashboardPutBoolean("DriverLED", false);
            mIsLEDOn = false;
            mDriverLED.set(Relay.Value.kOff);
        }
    }

    public synchronized boolean getVisionLampState()
    {
        return mIsLampOn;
    }

    public synchronized void setVisionLampOn()
    {
        if (!mIsLampOn)
        {
            mIsLampOn = true;
            dashboardPutBoolean("VisionLamp", mIsLampOn);
            mVisionLamp.set(Relay.Value.kForward);
        }
    }

    public synchronized void setVisionLampOff()
    {
        if (mIsLampOn)
        {
            mIsLampOn = false;
            dashboardPutBoolean("VisionLamp", mIsLampOn);
            mVisionLamp.set(Relay.Value.kOff);
        }
    }

    public synchronized void configureBlink(int blinkCount, double blinkDuration)
    {
        mBlinkDuration = blinkDuration;
        mBlinkCount = blinkCount;
        mTotalBlinkDuration = mBlinkCount * mBlinkDuration;
    }
    
    @Override
    public boolean checkSystem(String variant)
    {
        logNotice("checkSystem ---------------");
        logNotice("VisionLamp On");
        setVisionLampOn();
        Timer.delay(2.0);
        logNotice("VisionLamp Off");
        setVisionLampOff();
        return true;
    }
}
