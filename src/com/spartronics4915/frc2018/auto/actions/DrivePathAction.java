package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.control.Path;

/**
 * Drives the robot along the Path defined in the PathContainer object. The
 * action finishes once the robot reaches the
 * end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action
{

    private PathContainer mPathContainer;
    private Path mPath;
    private Drive mDrive = Drive.getInstance();

    public DrivePathAction(PathContainer p)
    {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
    }

    @Override
    public void start()
    {
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public boolean isFinished()
    {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void update()
    {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done()
    {
        mDrive.setVelocitySetpoint(0, 0);
    }

}
