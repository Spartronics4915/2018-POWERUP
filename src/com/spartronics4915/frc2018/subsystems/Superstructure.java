package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Timer;

/**
 * The superstructure subsystem is the overarching superclass containing all
 * components of the superstructure: climber, harvester, and articulated grabber,
 * and lifter.
 * 
 * The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like
 * the compressor, pressure sensor, and
 * hopper wall pistons.
 * 
 * Instead of interacting with subsystems like the feeder and intake directly,
 * the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like
 * the feeder and shooter.
 * 
 * @see LED
 * @see Subsystem
 */
public class Superstructure extends Subsystem
{

    static Superstructure mInstance = null;

    public static Superstructure getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private final LED mLED = LED.getInstance();
    private final ArticulatedGrabber mGrabber = ArticulatedGrabber.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final Harvester mHarvester = Harvester.getInstance();
    private final ScissorLift mLifter = ScissorLift.getInstance();

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState
    {
        IDLE
    };

    // Desired function from user
    public enum WantedState
    {
        IDLE
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    // State change timestamps are currently unused, but I'm keeping them
    // here because they're useful.
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    public boolean isDriveOnTarget()
    {
        return mDrive.isOnTarget() && mDrive.isAutoAiming();
    }

    private Loop mLoop = new Loop()
    {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                SystemState newState = mSystemState;
                switch (mSystemState)
                {
                    case IDLE:
                        newState = handleIdle(mStateChanged);
                        break;
                    default:
                        newState = SystemState.IDLE;
                }

                if (newState != mSystemState)
                {
                    Logger.notice("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                }
                else
                {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
        }
    };

    private SystemState handleIdle(boolean stateChanged)
    {
        if (stateChanged)
        {
            stop();
            mLED.setWantedState(LED.WantedState.OFF);
        }

        switch (mWantedState)
        {
            // Add states you want to be able to switch to from IDLE
            default:
                return SystemState.IDLE;
        }
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard()
    {
        // If we have any miscellaneous hardware that we put into this subsystem, it goes here
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
}
