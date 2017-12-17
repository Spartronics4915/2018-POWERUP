package com.team254.frc2017.auto.actions;

import com.team254.frc2017.RobotState;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.subsystems.Drive;
import com.team254.lib.util.math.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Resets the robot's current pose based on the starting pose stored in the pathContainer object.
 * 
 * @see PathContainer
 * @see Action
 * @see RunOnceAction
 */
public class ResetPoseFromPathAction extends RunOnceAction {

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce() {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        Drive.getInstance().setGyroAngle(startPose.getRotation());
    }
}
