package com.spartronics4915.frc2018.loops;

import com.spartronics4915.frc2018.Kinematics;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two
 * waypoints), gyroscope orientation, and velocity, among various other factors. Similar to a car's
 * odometer.
 */
public class RobotStateEstimator implements Loop {
  static RobotStateEstimator instance_ = new RobotStateEstimator();

  public static RobotStateEstimator getInstance() {
    return instance_;
  }

  RobotStateEstimator() {}

  RobotState robot_state_ = RobotState.getInstance();
  Drive drive_ = Drive.getInstance();
  double left_encoder_prev_distance_ = 0;
  double right_encoder_prev_distance_ = 0;

  @Override
  public synchronized void onStart(double timestamp) {
    left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
    right_encoder_prev_distance_ = drive_.getRightDistanceInches();
  }

  @Override
  public synchronized void onLoop(double timestamp) {
    final double left_distance = drive_.getLeftDistanceInches();
    final double right_distance = drive_.getRightDistanceInches();
    final Rotation2d gyro_angle = drive_.getGyroAngle();
    final Twist2d odometry_velocity =
        robot_state_.generateOdometryFromSensors(
            left_distance - left_encoder_prev_distance_,
            right_distance - right_encoder_prev_distance_,
            gyro_angle);
    final Twist2d predicted_velocity =
        Kinematics.forwardKinematics(
            drive_.getLeftVelocityInchesPerSec(),
            drive_.getRightVelocityInchesPerSec()); // DEBUG: Try using the overloaded function that
    // takes gyro delta v
    robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
    left_encoder_prev_distance_ = left_distance;
    right_encoder_prev_distance_ = right_distance;
  }

  @Override
  public void onStop(double timestamp) {
    // no-op
  }
}
