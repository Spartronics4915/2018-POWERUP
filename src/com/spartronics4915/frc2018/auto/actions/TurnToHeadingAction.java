package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.Logger;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class TurnToHeadingAction implements Action
{

    private Rotation2d mTargetHeading;
    private Drive mDrive = Drive.getInstance();

    public TurnToHeadingAction(Rotation2d heading)
    {
        mTargetHeading = heading;
    }

    @Override
    public void start()
    {
        Logger.notice("TurnToHeadingAction starting.");
        mDrive.setWantTurnToHeading(mTargetHeading);
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
        Logger.notice("TurnToHeadingAction done.");
    }

}
