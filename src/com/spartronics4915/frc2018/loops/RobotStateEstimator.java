package com.spartronics4915.frc2018.loops;

import com.spartronics4915.frc2018.Kinematics;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's
 * odometer.
 */
public class RobotStateEstimator implements Loop
{

    static RobotStateEstimator mInstance = null;

    public static RobotStateEstimator getInstance()
    {
        if(mInstance == null)
        {
            mInstance = new RobotStateEstimator();
        }
        return mInstance;
    }

    RobotStateEstimator()
    {
    }

    RobotState mRobotState = RobotState.getInstance();
    Drive mDrive = Drive.getInstance();
    double mLeftEncoderPrevDist = 0;
    double mRightEncoderPrevDist = 0;

    @Override
    public synchronized void onStart(double timestamp)
    {
        mLeftEncoderPrevDist = mDrive.getLeftDistanceInches();
        mRightEncoderPrevDist = mDrive.getRightDistanceInches();
    }

    @Override
    public synchronized void onLoop(double timestamp)
    {
        final double left_distance = mDrive.getLeftDistanceInches();
        final double right_distance = mDrive.getRightDistanceInches();
        final Rotation2d gyro_angle = mDrive.getGyroAngle();
        final Twist2d odometry_velocity = mRobotState.generateOdometryFromSensors(
                left_distance - mLeftEncoderPrevDist, right_distance - mRightEncoderPrevDist, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(mDrive.getLeftVelocityInchesPerSec(),
                mDrive.getRightVelocityInchesPerSec());
        mRobotState.addObservations(timestamp, odometry_velocity, predicted_velocity);
        mLeftEncoderPrevDist = left_distance;
        mRightEncoderPrevDist = right_distance;
    }

    @Override
    public void onStop(double timestamp)
    {
        // no-op
    }

}
