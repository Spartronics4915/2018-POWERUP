package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Timer;

public class DriveClosedLoopAction implements Action
{
    Drive mDrive = Drive.getInstance();
    String mClosedLoopMode;
    DriveSignal mSetpoint;
    double mTimeToRunSeconds;
    double mStartTimeSeconds;
    
    // mode: "velocity", "position"
    public DriveClosedLoopAction(String mode, DriveSignal setpoint, double durationSeconds)
    {
        mClosedLoopMode = mode;
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
    }

    @Override
    public void start()
    {
        Logger.notice("DriveClosedLoopAction " + mTimeToRunSeconds + " seconds");
        mStartTimeSeconds = Timer.getFPGATimestamp();
        if(mClosedLoopMode == "velocity")
            mDrive.setVelocitySetpoint(mSetpoint.getLeft(), mSetpoint.getRight());
        else
            mDrive.setRelativePosition(mSetpoint.getLeft(), mSetpoint.getRight());
    }
    
}
