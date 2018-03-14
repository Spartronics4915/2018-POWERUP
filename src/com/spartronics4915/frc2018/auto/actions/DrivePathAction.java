package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
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
    private String mStopMarker;

    public DrivePathAction(PathContainer p)
    {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mStopMarker = "";
    }
    
    public DrivePathAction(PathContainer p, String stopMarker) {
        this(p);
        mStopMarker = stopMarker;
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
        if (!mStopMarker.equals("") && mDrive.hasPassedMarker(mStopMarker))
            mDrive.forceDoneWithPath();
    }

    @Override
    public void done()
    {
        mDrive.setVelocitySetpoint(0, 0);
    }
}
