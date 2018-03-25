package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.frc2018.subsystems.LED.SystemState;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.drivers.LazySolenoid;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * The scissor lift is controlled by pneumatics. It has multiple set positions
 * and variable height. The key thing here is that its system state relates to
 * the highly variable position of the lifter.
 * 
 * TODO:  right now the method for calculating offsets presumes that the
 * default potentiometer value is near zero.  Since it appears that the
 * pot value is high at retracted position (with decreasing values as you 
 * go higher), we must modify the update strategy to target = mPotHome - defaultTarget.
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
    private static final int kDefaultRetractedOffset = 0;
    private static final int kDefaultSwitchOffset = 1550;
    private static final int kDefaultScaleOffset = 2051;
    private static final int kDefaultClimbOffset = 2294;
    private static final int kPotentiometerAllowedError = 50;
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
        RELEASING,
    }

    public enum WantedState
    {
        OFF, // == 0,  is off different from retracted?
        RETRACTED,
        SWITCH,
        SCALE,
        CLIMB,
        MANUALUP,
        MANUALDOWN,
        CLIMBING_RELEASE,
    }

    private SystemState mSystemState = SystemState.OFF;
    private WantedState mWantedState = WantedState.RETRACTED;
    private int[] mWantedStateMap = new int[WantedState.values().length]; // set in zeroSensors()
    private AnalogInput mPotentiometer;
    private int mMeasuredValue; // [0,4095]
    private int mPotentiometerHome = 2544;
    private LazySolenoid mRaiseSolenoid;
    private LazySolenoid mLowerSolenoid;
    private LazySolenoid mHoldSolenoid;
    private Timer mTimer;
    private boolean mHasReachedTarget = false;

    // Actuators and sensors should be initialized as private members with a value of null here

    private ScissorLift()
    {
        boolean success = true;

        try
        {
            mPotentiometer = new AnalogInput(Constants.kScissorHeightPotentiometerId);
            mRaiseSolenoid = new LazySolenoid(Constants.kScissorUpSolenoidId);
            mLowerSolenoid = new LazySolenoid(Constants.kScissorDownSolenoidId);
            mHoldSolenoid = new LazySolenoid(Constants.kScissorBrakeSolenoidId);

            mTimer = new Timer();

            success = mRaiseSolenoid.isValid() && mLowerSolenoid.isValid() &&
                    mHoldSolenoid.isValid();

            dashboardPutState(mSystemState.toString());
            dashboardPutWantedState(mWantedState.toString());
            
            // Initialize network tables during robotInit(), allows us to tweak values
            //  XXX: requires us to place best-known values into these values.
            dashboardPutNumber("Target1", kDefaultRetractedOffset);
            dashboardPutNumber("Target2", kDefaultSwitchOffset);
            dashboardPutNumber("Target3", kDefaultScaleOffset);
            dashboardPutNumber("Target4", kDefaultClimbOffset);
            zeroPotentiometer(); // this is where we establish our base potentiometer value
            // TODO: Is there a way to validate our potentiometer state? We can't detect
            //  wiring mishaps, since reading the analog pin will always return a value.

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
            synchronized (ScissorLift.this)
            {
                mWantedState = WantedState.OFF;
                mSystemState = SystemState.OFF;
                dashboardPutState(mSystemState.toString());
                dashboardPutWantedState(mWantedState.toString());
                zeroPotentiometer();
                //zeroSensors(); // we dont want called between autonomous and teleop
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (ScissorLift.this)
            {
                mMeasuredValue = mPotentiometer.getValue();
                SystemState newState = updateState();
                if (newState != mSystemState)
                {
                    mSystemState = newState;
                    dashboardPutState(mSystemState.toString());
                    logNotice(mSystemState.toString());
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
        mHasReachedTarget = false;
        mWantedState = wantedState;
        dashboardPutWantedState(wantedState.toString());
        logNotice("Wanted state to" + wantedState);
    }
    
    public synchronized WantedState getWantedState()
    {
        return mWantedState;
    }

    public synchronized boolean atTarget()
    {
        boolean matches = false;
        switch (mSystemState)
        {
            case OFF:
                matches = (mWantedState == WantedState.OFF);
                break;
            case RAISING:
                matches = false;
                break;
            case LOWERING:
                matches = false;
                break;
            case HOLDING:
                matches = (mWantedState == WantedState.RETRACTED || mWantedState == WantedState.OFF
                        || mWantedState == WantedState.SWITCH || mWantedState == WantedState.SCALE);
                break;
            case BRAKING:
                matches = false;
                break;
            case UNBRAKING:
                matches = false;
                break;
            default:
                matches = false;
                break;
        }
        return matches;
    }

    @Override
    public void outputToSmartDashboard()
    {
        dashboardPutNumber("Potentiometer", mPotentiometer.getAverageValue());
    }

    @Override
    public synchronized void stop()
    {
        mSystemState = SystemState.OFF;
        setWantedState(WantedState.OFF);
        mLowerSolenoid.set(false);
        mRaiseSolenoid.set(false);
        mHoldSolenoid.set(false);
    }

    public synchronized void zeroPotentiometer()
    {
        mPotentiometerHome = mPotentiometer.getValue();
    }

    private int getRetractedOffset()
    {
        //potentiometer numbers get smaller as Scissor Lift gets higher so we think of our targets as offsets
        return mPotentiometerHome - dashboardGetNumber("Target1", kDefaultRetractedOffset).intValue();
    }

    private int getSwitchOffset()
    {
        return mPotentiometerHome - dashboardGetNumber("Target2", kDefaultSwitchOffset).intValue(); 
    }

    private int getScaleOffset()
    {
        return mPotentiometerHome - dashboardGetNumber("Target3", kDefaultScaleOffset).intValue();
    }

    private int getClimbOffset()
    {
        return mPotentiometerHome - dashboardGetNumber("Target4", kDefaultClimbOffset).intValue();
    }
    
    @Override
    public void zeroSensors()
    {
        // we update our value map here based on smart dashboard values.
        // we could also auto-calibrate our 'zero" here if we're in a known position.
        mWantedStateMap[WantedState.OFF.ordinal()] = getRetractedOffset();
        mWantedStateMap[WantedState.RETRACTED.ordinal()] = getRetractedOffset();
        mWantedStateMap[WantedState.SWITCH.ordinal()] =  getSwitchOffset();
        mWantedStateMap[WantedState.SCALE.ordinal()] = getScaleOffset();
        mWantedStateMap[WantedState.CLIMB.ordinal()] = getClimbOffset();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    // based on the combination of wanted state, current state and the current potentiometer value,
    // transfer the current system state to another state.
    /*
     * Notes on solenoid controls (from Riyadth):
     * To clarify the brake solenoid: As the scissor lift rises to the point
     * where the software wants to stop it, the software should first engage
     * the brake solenoid, which will hold the scissor lift in place. The
     * software may want to continue to allow air in the lift solenoid for a
     * period of time afterwards, so that the scissor lift is pressurized
     * firmly. If we turn off the lift solenoid too soon there is a chance
     * that the lift will be "bouncy", and could go down a bit while
     * positioning. The brake only keeps the scissor from rising higher, and
     * does not prevent it from going lower.
     * After applying the brake, in order to move the scissor lift again, the
     * brake must be released, and then the scissor lift must be lowered a small
     * amount, even if the desire is to raise it up higher. This is because
     * there is a mechanical cam that is under tension, holding the lift from
     * going higher, until the lift is lowered slightly. Of course, if the lift
     * needs to be lowered anyway, it will be free to do so after the brake is
     * released (ie, no small motion required first).
     */
    private SystemState updateState()
    {
        int targetValue = mWantedStateMap[mWantedState.ordinal()];
        SystemState nextState = mSystemState;
        mMeasuredValue = mPotentiometer.getAverageValue();
        if (mWantedState == WantedState.MANUALUP) // TODO: Try to implement a jog function
        {

        }
        else if (mWantedState == WantedState.MANUALDOWN)
        {

        }
        else if (mWantedState == WantedState.OFF) // STOP 
        {
            mRaiseSolenoid.set(false);
            mLowerSolenoid.set(false);
            mHoldSolenoid.set(false);
            nextState = SystemState.OFF;
        }
        else if (mWantedState == WantedState.CLIMBING_RELEASE)
        {
            mRaiseSolenoid.set(false);
            mLowerSolenoid.set(false);
            mHoldSolenoid.set(false);
            nextState = SystemState.RELEASING;
        }
        else if (Util.epsilonGreaterThan(mMeasuredValue, targetValue, kPotentiometerAllowedError) && !mHasReachedTarget)
        {
            // we're below target position, let's raise
            if (mSystemState != SystemState.RAISING)
            {
                if (mSystemState == SystemState.HOLDING || mSystemState == SystemState.BRAKING)
                {
                    mTimer.reset();
                    mTimer.start();
                    mHoldSolenoid.set(false);
                    mLowerSolenoid.set(true); // brake released, must go down a bit before up
                    mRaiseSolenoid.set(false);
                    nextState = SystemState.UNBRAKING;
                }
                else if (mSystemState == SystemState.UNBRAKING)
                {
                    if (mTimer.hasPeriodPassed(kUnbrakeTimePeriod))
                    {
                        mHoldSolenoid.set(false);
                        mLowerSolenoid.set(false);
                        mRaiseSolenoid.set(true);
                        nextState = SystemState.RAISING;
                    }
                }
                else
                {
                    mLowerSolenoid.set(false);
                    mRaiseSolenoid.set(true);
                    mHoldSolenoid.set(false);
                    nextState = SystemState.RAISING;
                }
            }
        }
        else if (Util.epsilonLessThan(mMeasuredValue, targetValue, kPotentiometerAllowedError) && !mHasReachedTarget)
        {
            // we're above target position, let's lower
            if (mSystemState != SystemState.LOWERING)
            {
                if (mSystemState == SystemState.HOLDING || mSystemState == SystemState.BRAKING)
                {
                    mHoldSolenoid.set(false); // brake release
                    mLowerSolenoid.set(true); // we're going down, proceed immediately to LOWERING
                    mRaiseSolenoid.set(false);
                    nextState = SystemState.LOWERING;
                }
                else
                {
                    mLowerSolenoid.set(true); // we're going down, proceed immediately to LOWERING
                    mRaiseSolenoid.set(false);
                    mHoldSolenoid.set(false);
                    nextState = SystemState.LOWERING;
                }
            }
            // else nextState = SystemState.LOWERING // (which was current state);
        }
        else
        {
            mHasReachedTarget = true;
            // we're on target
            if (mSystemState != SystemState.HOLDING)
            {
                if (mSystemState != SystemState.BRAKING)
                {
                    mTimer.reset();
                    mTimer.start();
                    mHoldSolenoid.set(true); // brake set, allow system to "settle" before HOLDING
                    nextState = SystemState.BRAKING;
                }
                else if (mTimer.hasPeriodPassed(kBrakeTimePeriod))
                {
                    mLowerSolenoid.set(false);
                    mRaiseSolenoid.set(true);
                    nextState = SystemState.HOLDING;
                }
                // else nextState = SystemState.BRAKING // (which was current state);
            }
            // else nextState = SystemState.HOLDING // (which was current state);
        }
        return nextState;
    }

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
                logNotice("  raise solenoid state " + mRaiseSolenoid.get());
                logNotice("  lower solenoid state " + mLowerSolenoid.get());
                logNotice("  hold solenoid state " + mHoldSolenoid.get());
                logNotice("  potentiometer value " + mPotentiometer.getValue());
            }
            if (variant.equals("raise") || allTests)
            {
                logNotice("raise check -----");
                logNotice("  raise: false (2sec)");
                mRaiseSolenoid.set(false);
                Timer.delay(2.0);
                logNotice("  pot: " + mPotentiometer.getValue());

                logNotice("  raise: true (3.5 sec)");
                mRaiseSolenoid.set(true);
                ;
                Timer.delay(3.5);
                logNotice("  pot: " + mPotentiometer.getValue());

                logNotice("  raise: false (3.5 sec)");
                mRaiseSolenoid.set(false);
                Timer.delay(3.5);
                logNotice("  pot: " + mPotentiometer.getValue());
            }
            if (variant.equals("lower") || allTests)
            {
                logNotice("lower check -----");
                logNotice("  lower: false (2sec)");
                mLowerSolenoid.set(false);
                Timer.delay(2.0);
                logNotice("  pot: " + mPotentiometer.getValue());

                logNotice("  lower: true (.25 sec)");
                mLowerSolenoid.set(true);
                Timer.delay(.25);
                logNotice("  pot: " + mPotentiometer.getValue());

                logNotice("  lower: false (.25 sec)");
                mLowerSolenoid.set(false);
                Timer.delay(.25);
                logNotice("  pot: " + mPotentiometer.getValue());
            }
            if (variant.equals("brake") || allTests)
            {
                logNotice("brake check -----");
                logNotice("  brake: false (3.5sec)");
                mHoldSolenoid.set(false);
                Timer.delay(3.5);
                logNotice("  pot: " + mPotentiometer.getValue());

                logNotice("  brake: true (.25 sec)");
                mHoldSolenoid.set(true);
                Timer.delay(.25);
                logNotice("  pot: " + mPotentiometer.getValue());

                logNotice("  brake: false (.25 sec)");
                mHoldSolenoid.set(false);
                Timer.delay(.25);
                logNotice("  pot: " + mPotentiometer.getValue());
            }
        }

        catch (Throwable e)
        {
            success = false;
            logException("checkSystem", e);
        }

        return success;
    }
}
