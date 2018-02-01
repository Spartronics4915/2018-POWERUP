package com.spartronics4915.frc2018;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.spartronics4915.frc2018.GoalTracker.TrackReport;
import com.spartronics4915.frc2018.vision.TargetInfo;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;
import com.spartronics4915.lib.util.math.Twist2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout
 * the match. A coordinate frame is simply a
 * point and direction in space that defines an (x,y) coordinate system.
 * Transforms (or poses) keep track of the spatial
 * relationship between different frames.
 *
 * Robot frames of interest (from parent to child):
 *
 * 1. Field frame: origin is where the robot is turned on
 *
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 *
 * 3. Camera frame: origin is the center of the camera imager relative to the
 * robot base.
 *
 * 4. Goal frame: origin is the center of the boiler (note that orientation in
 * this frame is arbitrary). Also note that
 * there can be multiple goal frames.
 *
 * As a kinematic chain with 4 frames, there are 3 transforms of interest:
 *
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably
 * drift, but is usually accurate over short time periods.
 *
 * 2. Vehicle-to-camera: This is a constant.
 *
 * 3. Camera-to-goal: This is a pure translation, and is measured by the vision
 * system.
 */

public class RobotState
{

    private static RobotState instance_ = null;

    public static RobotState getInstance()
    {
        if(instance_ == null)
        {
            instance_ = new RobotState();
        }
        return instance_;
    }

    private static final int kObservationBufferSize = 100;

    private static final RigidTransform2d kVehicleToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> mFieldToVehicle;
    private Twist2d mVehicleVelocityPredicted;
    private Twist2d mVehicleVelocityMeasured;
    private double mDistanceDriven;
    private GoalTracker mGoalTracker;
    private Rotation2d mCameraPitchCorrection;
    private Rotation2d mCameraYawCorrection;
    private double mDifferentialHeight;
    private ShooterAimingParameters mCachedShooterAimingParameters = null;

    private RobotState()
    {
        reset(0, new RigidTransform2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle)
    {
        mFieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        mFieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        mVehicleVelocityPredicted = Twist2d.identity();
        mVehicleVelocityMeasured = Twist2d.identity();
        mGoalTracker = new GoalTracker();
        mCameraPitchCorrection = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        mCameraYawCorrection = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
        // mDifferentialHeight = Constants.kBoilerTargetTopHeight - Constants.kCameraZOffset;
        mDistanceDriven = 0.0;
    }

    public synchronized void resetDistanceDriven()
    {
        mDistanceDriven = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized RigidTransform2d getFieldToVehicle(double timestamp)
    {
        return mFieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle()
    {
        return mFieldToVehicle.lastEntry();
    }

    public synchronized RigidTransform2d getPredictedFieldToVehicle(double lookahead_time)
    {
        return getLatestFieldToVehicle().getValue()
                .transformBy(RigidTransform2d.exp(mVehicleVelocityPredicted.scaled(lookahead_time)));
    }

    public synchronized RigidTransform2d getFieldToCamera(double timestamp)
    {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToCamera);
    }

    public synchronized List<RigidTransform2d> getCaptureTimeFieldToGoal()
    {
        List<RigidTransform2d> rv = new ArrayList<>();
        for (TrackReport report : mGoalTracker.getTracks())
        {
            rv.add(RigidTransform2d.fromTranslation(report.field_to_goal));
        }
        return rv;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2d observation)
    {
        mFieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d measured_velocity,
            Twist2d predicted_velocity)
    {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        mVehicleVelocityMeasured = measured_velocity;
        mVehicleVelocityPredicted = predicted_velocity;
    }

    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update)
    {
        List<Translation2d> field_to_goals = new ArrayList<>();
        RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
        if (!(vision_update == null || vision_update.isEmpty()))
        {
            for (TargetInfo target : vision_update)
            {
                double ydeadband = (target.getY() > -Constants.kCameraDeadband
                        && target.getY() < Constants.kCameraDeadband) ? 0.0 : target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * mCameraYawCorrection.cos() + ydeadband * mCameraYawCorrection.sin();
                double yyaw = ydeadband * mCameraYawCorrection.cos() - target.getX() * mCameraYawCorrection.sin();
                double zyaw = target.getZ();

                // Compensate for camera pitch
                double xr = zyaw * mCameraPitchCorrection.sin() + xyaw * mCameraPitchCorrection.cos();
                double yr = yyaw;
                double zr = zyaw * mCameraPitchCorrection.cos() - xyaw * mCameraPitchCorrection.sin();

//                // find intersection with the goal
//                if (zr > 0)
//                {
//                    double scaling = mDifferentialHeight / zr;
//                    double distance = Math.hypot(xr, yr) * scaling + Constants.kBoilerRadius;
//                    Rotation2d angle = new Rotation2d(xr, yr, true);
//                    field_to_goals.add(field_to_camera
//                            .transformBy(RigidTransform2d
//                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
//                            .getTranslation());
//                }
            }
        }
        synchronized (this)
        {
            mGoalTracker.update(timestamp, field_to_goals);
        }
    }

    public synchronized Optional<ShooterAimingParameters> getCachedAimingParameters()
    {
        return mCachedShooterAimingParameters == null ? Optional.empty() : Optional.of(mCachedShooterAimingParameters);
    }

    public synchronized Optional<ShooterAimingParameters> getAimingParameters()
    {
        List<TrackReport> reports = mGoalTracker.getTracks();
        if (!reports.isEmpty())
        {
            TrackReport report = reports.get(0);
            Translation2d robot_to_goal = getLatestFieldToVehicle().getValue().getTranslation().inverse()
                    .translateBy(report.field_to_goal);
            Rotation2d robot_to_goal_rotation = Rotation2d
                    .fromRadians(Math.atan2(robot_to_goal.y(), robot_to_goal.x()));

            ShooterAimingParameters params = new ShooterAimingParameters(robot_to_goal.norm(), robot_to_goal_rotation,
                    report.latest_timestamp, report.stability);
            mCachedShooterAimingParameters = params;

            return Optional.of(params);
        }
        else
        {
            return Optional.empty();
        }
    }

    public synchronized void resetVision()
    {
        mGoalTracker.reset();
        mCachedShooterAimingParameters = null;
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle)
    {
        final RigidTransform2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
        mDistanceDriven += delta.dx;
        return delta;
    }

    public synchronized double getDistanceDriven()
    {
        return mDistanceDriven;
    }

    public synchronized Twist2d getPredictedVelocity()
    {
        return mVehicleVelocityPredicted;
    }

    public synchronized Twist2d getMeasuredVelocity()
    {
        return mVehicleVelocityMeasured;
    }

    public void outputToSmartDashboard()
    {
        RigidTransform2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putString("RobotState/pose",  
                            "" + odometry.getTranslation().x() + 
                            " " + odometry.getTranslation().y() +
                            " " + odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("RobotState/velocity", mVehicleVelocityMeasured.dx);
        SmartDashboard.putNumber("RobotState/field_degrees", getLatestFieldToVehicle().getValue().getRotation().getDegrees());
        List<RigidTransform2d> poses = getCaptureTimeFieldToGoal();
        for (RigidTransform2d pose : poses)
        {
            // Only output first goal
            SmartDashboard.putNumber("RobotState/goal_pose_x", pose.getTranslation().x());
            SmartDashboard.putNumber("RobotState/goal_pose_y", pose.getTranslation().y());
            break;
        }
        Optional<ShooterAimingParameters> aiming_params = getCachedAimingParameters();
        if (aiming_params.isPresent())
        {
            SmartDashboard.putNumber("RobotState/goal_range", aiming_params.get().getRange());
            SmartDashboard.putNumber("RobotState/goal_theta", aiming_params.get().getRobotToGoal().getDegrees());
        }
        else
        {
            SmartDashboard.putNumber("RobotState/goal_range", 0.0);
            SmartDashboard.putNumber("RobotState/goal_theta", 0.0);
        }
    }
}
