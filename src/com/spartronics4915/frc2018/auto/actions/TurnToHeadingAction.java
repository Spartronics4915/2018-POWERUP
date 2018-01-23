package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.DriveSys;
import com.spartronics4915.lib.util.math.Rotation2d;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class TurnToHeadingAction implements Action
{

    private Rotation2d mTargetHeading;
    private DriveSys mDrive = DriveSys.getInstance();

    public TurnToHeadingAction(Rotation2d heading)
    {
        mTargetHeading = heading;
    }

    @Override
    public boolean isFinished()
    {
        return mDrive.isDoneWithTurn();
    }

    @Override
    public void update()
    {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done()
    {
    }

    @Override
    public void start()
    {
        mDrive.setWantTurnToHeading(mTargetHeading);
    }
}
