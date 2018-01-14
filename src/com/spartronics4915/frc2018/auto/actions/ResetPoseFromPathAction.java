package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.math.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Resets the robot's current pose based on the starting pose stored in the
 * pathContainer object.
 * 
 * @see PathContainer
 * @see Action
 * @see RunOnceAction
 */
public class ResetPoseFromPathAction extends RunOnceAction
{

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer)
    {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce()
    {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        Drive.getInstance().setGyroAngle(startPose.getRotation());
    }
}
