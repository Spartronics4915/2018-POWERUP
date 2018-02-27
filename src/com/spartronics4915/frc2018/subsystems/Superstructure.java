package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * The superstructure subsystem is the overarching superclass containing all
 * components of the superstructure: climber, harvester, and articulated
 * grabber,
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

    private LED mLED = null;
    private ArticulatedGrabber mGrabber = null;
    private Climber mClimber = null;
    private Harvester mHarvester = null;
    private ScissorLift mLifter = null;

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Internal state of the system
    public enum SystemState
    {
        IDLE,
        OPENING_HARVESTER, // Transfer cube from harvester to scissor
        GRABBING_ARTICULATED_GRABBER,
        TRANSPORTING_ARTICULATED_GRABBER,
        TRANSPORTING_ARTICULATED_GRABBER_DELAY,
        RELEASING_SCISSOR, // Climb
        CLIMBING,
        DRIVE_CUBE, // turn to cube
        VISION_HARVEST // drive to, and harvest cube
    };

    // Desired function from user
    public enum WantedState
    {
        IDLE,
        TRANSFER_CUBE_TO_GRABBER,
        CLIMB,
        VISION_AQUIRE_CUBE, // drive to, and acquire the cube
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    // State change timestamps are currently unused, but I'm keeping them
    // here because they're potentially useful.
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Timer mTimer = new Timer();

    private final double kMatchDurationSeconds = 135;
    private final double kEndgameDurationSeconds = 30;
    private final double kFinishGrabAfterSeconds = 0.8;

    private Superstructure()
    {
        mLED = LED.getInstance();
        mGrabber = ArticulatedGrabber.getInstance();
        mClimber = Climber.getInstance();
        mHarvester = Harvester.getInstance();
        mLifter = ScissorLift.getInstance();
    }
    
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
                        switch(mWantedState)
                        {
                            case TRANSFER_CUBE_TO_GRABBER:
                                newState = SystemState.OPENING_HARVESTER;
                                break;
                            case CLIMB:
                                newState = SystemState.RELEASING_SCISSOR;
                                break;
                            default: // either idle or unimplemented
                                break;
                        }
                        break;
                    case OPENING_HARVESTER: // Transfer cube from harvester to scissor
                        if (mHarvester.getWantedState() != Harvester.WantedState.OPEN)
                            mHarvester.setWantedState(Harvester.WantedState.OPEN);
                        if (mWantedState == WantedState.TRANSFER_CUBE_TO_GRABBER)
                        {
                            if (mHarvester.atTarget())
                                newState = SystemState.GRABBING_ARTICULATED_GRABBER;
                        }
                        else
                            newState = SystemState.IDLE;
                        break;
                    case GRABBING_ARTICULATED_GRABBER:
                        if (mGrabber.getWantedState() != ArticulatedGrabber.WantedState.GRAB_CUBE)
                            mGrabber.setWantedState(ArticulatedGrabber.WantedState.GRAB_CUBE);
                        if (mWantedState == WantedState.TRANSFER_CUBE_TO_GRABBER)
                        {
                            if (mGrabber.atTarget())
                            {
                                newState = SystemState.TRANSPORTING_ARTICULATED_GRABBER_DELAY;
                                mTimer.reset();
                                mTimer.start();
                           }
                        }
                        else
                            newState = SystemState.IDLE;
                        break;
                    case TRANSPORTING_ARTICULATED_GRABBER_DELAY:
                        if(mWantedState == WantedState.TRANSFER_CUBE_TO_GRABBER)
                        {
                            if(mTimer.hasPeriodPassed(kFinishGrabAfterSeconds))
                            {
                                newState = SystemState.TRANSPORTING_ARTICULATED_GRABBER;
                            }
                        }
                        else
                            newState = SystemState.IDLE;
                        break;
                    case TRANSPORTING_ARTICULATED_GRABBER:
                        if (mWantedState == WantedState.TRANSFER_CUBE_TO_GRABBER)
                        {
                           if(mGrabber.getWantedState() != ArticulatedGrabber.WantedState.PREPARE_DROP)
                               mGrabber.setWantedState(ArticulatedGrabber.WantedState.PREPARE_DROP);
                            if (mGrabber.atTarget())
                            {
                                mHarvester.setWantedState(Harvester.WantedState.DISABLE);
                                mWantedState = WantedState.IDLE;
                                newState = SystemState.IDLE; // Done
                            }
                        }
                        else
                            newState = SystemState.IDLE;
                        break;
                    case RELEASING_SCISSOR: // Climb
                        if (mLifter.getWantedState() != ScissorLift.WantedState.OFF)
                            mLifter.setWantedState(ScissorLift.WantedState.OFF);
                        if (mWantedState == WantedState.CLIMB)
                        {
                            if (mLifter.atTarget())
                                newState = SystemState.CLIMBING;
                        }
                        else
                            newState = SystemState.IDLE;
                        break;
                    case CLIMBING:
                        if (mClimber.getWantedState() != Climber.WantedState.CLIMB)
                            mClimber.setWantedState(Climber.WantedState.CLIMB);
                        else
                        {
                            mWantedState = WantedState.IDLE;
                            newState = SystemState.IDLE; // Done
                        }
                        break;
                    case DRIVE_CUBE: //The spinning to a cube
                        if (mDrive.getDriveState() != Drive.DriveControlState.FIND_CUBE)
                        {
                            mDrive.setWantSearchForCube(); //Begin Searching for cube
                            // IF THIS IS NOT CALLED REPEADLY, THEN THE FOLLOWING PART NEEDS TO BE CHANGED
                            // The other solution is for wantSearchForCube to tell someone when he is done, ATM he just default back to OPEN_LOOP when done.
                            if (mDrive.getDriveState() == Drive.DriveControlState.OPEN_LOOP) // (Or whatever the state is for standard driver operation)
                            {
                                newState = SystemState.VISION_HARVEST;

                            }
                        }
                        else
                            newState = SystemState.IDLE;
                    case VISION_HARVEST: //Drive forward until we have the cube, then dump controls back to the driver
                        if (mHarvester.getWantedState() != Harvester.WantedState.HARVEST)
                        {
                            mHarvester.setWantedState(Harvester.WantedState.HARVEST);
                            // IN TANDEM WITH THIS: mDrive.setWantDriveForwardSlowly();
                            // We return to default drive control when the harvester has a cube.
                        }
                        // I don't know if the following 'if' statement is nested in the previous if statement, or fine in its current state
                        if (mHarvester.atTarget())
                        {
                            newState = SystemState.IDLE; // Done
                                                         // Return the robot to driver control
                        }
                        
                    default:
                        newState = defaultStateTransfer();
                }
                if(mSystemState != newState)
                {
                    if(newState == SystemState.IDLE)
                    {
                        // need to reset subsystems to an idle?
                    }
                }
                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
        }
    };

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case IDLE:
                newState = SystemState.IDLE;
                break;
            case TRANSFER_CUBE_TO_GRABBER:
                newState = SystemState.OPENING_HARVESTER; // First state
                break;
            case CLIMB:
                //                if (DriverStation.getInstance().getMatchTime() < kMatchDurationSeconds - kEndgameDurationSeconds) // Don't extend the scissor if we're not in the endgame
                //                    return; This is commented out to make testing easier. Re-add it once this is verified.
                newState = SystemState.RELEASING_SCISSOR; // First state
                break;
            case VISION_AQUIRE_CUBE:
                // Begin the spin
                newState = SystemState.DRIVE_CUBE;
            default:
                newState = SystemState.IDLE;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        logNotice("Wanted state to " + wantedState.toString());
        mWantedState = wantedState;
    }

    /**
     * This is a bit like the atTarget methods of many other subsystem.
     * It's called something different because Superstructure is fundamentally
     * different from other subsystems.
     */
    public synchronized boolean isIdling()
    {
        return mSystemState == SystemState.IDLE;
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

    @Override
    public boolean checkSystem(String variant)
    {
        logNotice("checkSystem not implemented");
        return false;
    }
}
