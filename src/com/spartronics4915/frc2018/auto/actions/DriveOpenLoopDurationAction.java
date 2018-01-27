package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Timer;

public class DriveOpenLoopDurationAction implements Action
{
    Drive mDrive = Drive.getInstance();
    DriveSignal mSetpoint;
    double mTimeToRunSeconds;
    double mStartTimeSeconds;
    
    public DriveOpenLoopDurationAction(DriveSignal setpoint, double durationSeconds) {
        mSetpoint = setpoint;
        mTimeToRunSeconds = durationSeconds;
    }
    
    @Override
    public boolean isFinished()
    {
        return Timer.getFPGATimestamp() - mStartTimeSeconds >= mTimeToRunSeconds;
    }

    @Override
    public void update()
    {
    }

    @Override
    public void done()
    {
        mDrive.setOpenLoop(new DriveSignal(0, 0));
    }

    @Override
    public void start()
    {
        mStartTimeSeconds = Timer.getFPGATimestamp();
        mDrive.setOpenLoop(mSetpoint);
    }
    
}
