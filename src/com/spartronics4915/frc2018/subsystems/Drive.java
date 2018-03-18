package com.spartronics4915.frc2018.subsystems;

import java.util.Optional;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.Kinematics;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.ShooterAimingParameters;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.control.Lookahead;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.control.PathFollower;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Drive;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Twist2d;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons,
 * one solenoid and 2 pistons to shift gears, and a Pigeon IMU board.
 * The Drive subsystem has several control methods including open loop,
 * velocity control, and position control. The Drive subsystem also has
 * several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 *
 * @see Subsystem.java
 */
public class Drive extends Subsystem
{

    private static Drive mInstance = null;

    private static final int kPositionControlSlot = 0;
    private static final int kVelocityControlSlot = 1;
    private static final double kOpenLoopRampRate = .5;
    private static final double kOpenLoopNominalOutput = 0.0; // fwd & rev
    private static final double kOpenLoopPeakOutput = 1; // fwd: 1, rev: -1
    private static final String kTargetVelL = "targetVelL";
    private static final String kTargetVelR = "targetVelR";

    public static Drive getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Drive();
        }
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState
    {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        POSITION_SETPOINT, // position PID control
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // TransitoryState - Goal is a target expressed in field coordinates
        TURN_TO_ROBOTANGLE, // TURN_TO_HEADING, but operates in robot-relative coords
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH, // drive forwards until we are at optimal shooting distance
        FIND_CUBE //Spin until we find the cube!
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private TalonSRX4915Drive mMotorGroup = null;

    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;
    private NetworkTableEntry mVisionTargetAngleEntry = null;
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;
    private boolean mIsSaturated = false;

    // Logging
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;

    // mLoop not registered when we're not initialized
    private final Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Drive.this)
            {
                // XXX: looper received onStart method, unclear why we're jumping
                //  into velocity mode here? (without consulting mDriveControlState)
                logNotice("onStart " + mDriveControlState);
                setOpenLoop(DriveSignal.NEUTRAL);
                setVelocitySetpoint(0, 0);
                if (!mMotorGroup.hasIMU())
                {
                    logError("IMU in non-ready state. Is it plugged in?");
                    return;
                }
                mMotorGroup.setGyroAngle(0);
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Drive.this)
            {
                switch (mDriveControlState)
                {
                    case OPEN_LOOP:
                    case VELOCITY_SETPOINT:
                    case POSITION_SETPOINT:
                        return;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null)
                        {
                            updatePathFollower(timestamp);
                            mCSVWriter.add(mPathFollower.getDebug());
                        }
                        return;
                    case TURN_TO_HEADING:
                        updateTurnToFieldHeading(timestamp);
                        return;
                    case TURN_TO_ROBOTANGLE:
                        //updateTurnToRobotHeading(timestamp);
                        logWarning("TURN_TO_ROBOTANGLE unimplemented");
                        return;
                    case DRIVE_TOWARDS_GOAL_COARSE_ALIGN:
                        updateDriveTowardsGoalCoarseAlign(timestamp);
                        return;
                    case DRIVE_TOWARDS_GOAL_APPROACH:
                        updateDriveTowardsGoalApproach(timestamp);
                        return;
                    case FIND_CUBE:
                        searchForCube(timestamp);
                        return;
                    default:
                        logError("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
            mCSVWriter.flush();
        }
    };

    private Drive()
    {
        final NetworkTable table =
                NetworkTableInstance.getDefault().getTable(Constants.kVisionTableName);
        mVisionTargetAngleEntry = table.getEntry(Constants.kVisionTargetAngleName);
        mVisionTargetAngleEntry.forceSetNumber(0);

        // encoder phase must match output sense or PID will spiral out of control
        mMotorGroup = new TalonSRX4915Drive(Constants.kDriveWheelDiameterInches,
                Constants.kEncoderCodesPerRev,
                Constants.kLeftDriveMasterId,
                Constants.kLeftDriveSlaveId,
                Constants.kRightDriveMasterId,
                Constants.kRightDriveSlaveId,
                Constants.kDriveIMUTalonId,
                TalonSRX4915Drive.Config.kLeftNormalRightInverted);

        if (mMotorGroup.isInitialized())
        {

            mMotorGroup.beginOpenLoop(kOpenLoopRampRate,
                    kOpenLoopNominalOutput, kOpenLoopPeakOutput);
            if (!mMotorGroup.hasIMU())
                logError("Could not detect the IMU. Is it plugged in?");
            reloadGains();
            mMotorGroup.enableBraking(true);
            logInitialized(true);
        }
        else
        {
            logInitialized(false);
        }

        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
    }

    @Override
    public void registerEnabledLoops(Looper in)
    {
        if (!this.isInitialized())
            return;

        in.register(mLoop);
    }

    /**
     * drives with open-loop control mode, called frequently!
     */
    public synchronized void setOpenLoop(DriveSignal signal)
    {
        if (!this.isInitialized())
            return;
        if (mDriveControlState != DriveControlState.OPEN_LOOP)
        {
            configureTalonsForOpenLoop();
        }
        mMotorGroup.driveOpenLoop(signal.getLeft(), signal.getRight());
    }

    @Override
    public synchronized void stop()
    {
        this.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void outputToSmartDashboard()
    {
        if (!this.isInitialized())
            return;
        mMotorGroup.outputToSmartDashboard();
        synchronized (this)
        {
            dashboardPutState(mDriveControlState.toString());
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
            {
                dashboardPutNumber("CTE", mPathFollower.getCrossTrackError());
                dashboardPutNumber("ATE", mPathFollower.getAlongTrackError());
            }
            if (mDriveControlState != DriveControlState.OPEN_LOOP)
                dashboardPutBoolean("onTarget", isOnTarget());
        }
    }

    public synchronized void resetEncoders()
    {
        if (!this.isInitialized())
            return;
        mMotorGroup.resetEncoders(false/* resetYaw */);
    }

    @Override
    public void zeroSensors()
    {
        if (!this.isInitialized())
            return;
        mMotorGroup.resetEncoders(true/* resetYaw */);
    }

    public synchronized Rotation2d getGyroAngle()
    {
        if (!this.isInitialized())
            return new Rotation2d();
        return Rotation2d.fromDegrees(mMotorGroup.getGyroAngle());
        // Rotation2d normalizes between -180 and 180 automatically
    }

    public synchronized void setGyroAngle(Rotation2d rot)
    {
        if (!this.isInitialized())
            return;
        mMotorGroup.setGyroAngle(rot.getDegrees());
    }

    /**
     * Start up velocity mode. Note that other control states may imply
     * the underlying speedControl mode but are initialized through a
     * different code path. ie: only call this method if you want direct
     * control over the velocity setpoint.
     *
     * @param leftIPS
     * @param rightIPS
     */
    public synchronized void setVelocitySetpoint(double leftIPS, double rightIPS)
    {
        if (!this.isInitialized())
            return;
        if (mDriveControlState != DriveControlState.VELOCITY_SETPOINT)
        {
            configureTalonsForSpeedControl();
            mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        }
        updateVelocitySetpoint(leftIPS, rightIPS);
    }

    /**
     * Start up position mode. Note that other control states may imply
     * the underlying positionControl mode but are initialized through a
     * different code path. ie: only call this method if you want direct
     * control over the position setpoint.
     *
     * @param leftInches
     * @param rightInches
     */
    public synchronized void setPositionSetpoint(double leftInches, double rightInches)
    {
        if (!this.isInitialized())
            return;
        if (mDriveControlState != DriveControlState.POSITION_SETPOINT)
        {
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.POSITION_SETPOINT;
        }
        updatePositionSetpoint(leftInches, rightInches);
    }

    /**
     * Startup in position mode with a setpoint expressed relative to
     * the current position. NB: this call should only be made if
     * the current position is a reliably stable value.
     * 
     * @param leftInches
     * @param rightInches
     */
    public synchronized void setRelativePosition(double leftInches, double rightInches)
    {
        if (!this.isInitialized())
            return;
        setPositionSetpoint(mMotorGroup.getLeftDistanceInches() + leftInches,
                mMotorGroup.getRightDistanceInches() + rightInches);
    }

    public double getLeftDistanceInches()
    {
        return mMotorGroup.getLeftDistanceInches();
    }

    public double getRightDistanceInches()
    {
        return mMotorGroup.getRightDistanceInches();
    }

    public double getLeftVelocityInchesPerSec()
    {
        return mMotorGroup.getLeftVelocityInchesPerSec();
    }

    public double getRightVelocityInchesPerSec()
    {
        return mMotorGroup.getRightVelocityInchesPerSec();
    }

    private void configureTalonsForOpenLoop()
    {
        if (!this.isInitialized())
            return;
        logNotice("beginOpenLoop");
        mMotorGroup.beginOpenLoop(kOpenLoopRampRate, kOpenLoopNominalOutput, kOpenLoopPeakOutput);
        mMotorGroup.enableBraking(true); // drivers like to stop on a dime
        mDriveControlState = DriveControlState.OPEN_LOOP;
        dashboardPutNumber(kTargetVelL, 0);
        dashboardPutNumber(kTargetVelR, 0);
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl()
    {
        if (!this.isInitialized())
            return;
        if (!usesTalonVelocityControl(mDriveControlState))
        {
            // We entered a velocity control state.
            logNotice("beginSpeedControl");
            mMotorGroup.resetIntegralAccumulator();
            mMotorGroup.beginClosedLoopVelocity(kVelocityControlSlot,
                    Constants.kDriveHighGearNominalOutput);
            mMotorGroup.enableBraking(true);
        }
    }

    /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl()
    {
        if (!this.isInitialized())
            return;
        if (!usesTalonPositionControl(mDriveControlState))
        {
            // We entered a position control state.
            logNotice("beginPositionControl");
            mMotorGroup.resetIntegralAccumulator();
            mMotorGroup.beginClosedLoopPosition(kPositionControlSlot,
                    Constants.kDriveLowGearNominalOutput,
                    Constants.kDriveLowGearMaxVelocity,
                    Constants.kDriveLowGearMaxAccel);

            // XXX: need to disable voltage compensation ramp rate
            // mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
            // mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);

            mMotorGroup.enableBraking(true);
            dashboardPutNumber(kTargetVelL, 0);
            dashboardPutNumber(kTargetVelR, 0);
        }
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     *
     * @param leftIPS
     * @param rightIPS
     */
    private synchronized void updateVelocitySetpoint(double leftIPS,
            double rightIPS)
    {
        if (!this.isInitialized())
            return;
        if (usesTalonVelocityControl(mDriveControlState))
        {
            final double max_desired =
                    Math.max(Math.abs(leftIPS), Math.abs(rightIPS));
            if (max_desired > Constants.kDriveHighGearMaxSetpoint)
            {
                final double scale = Constants.kDriveHighGearMaxSetpoint / max_desired;
                if (!mIsSaturated)
                    this.logWarning("drive control is saturated");
                mIsSaturated = true;
                rightIPS *= scale;
                leftIPS *= scale;
            }
            else
            {
                if (mIsSaturated)
                    this.logNotice("drive control no longer saturated");
                mIsSaturated = false;
            }

            dashboardPutNumber(kTargetVelL, leftIPS);
            dashboardPutNumber(kTargetVelR, rightIPS);
            mMotorGroup.driveVelocityInchesPerSec(leftIPS, rightIPS);
        }
        else
        {
            logError("Hit a bad velocity control state");
            dashboardPutNumber(kTargetVelL, 0);
            dashboardPutNumber(kTargetVelR, 0);
            mMotorGroup.driveVelocityInchesPerSec(0, 0); // XXX: should we just mDrive.stop()?
        }
    }

    /**
     * Adjust position setpoint (if already in position mode)
     *
     * @param leftPosInches
     * @param rightPosInches
     */
    private synchronized void updatePositionSetpoint(double leftPosInches,
            double rightPosInches)
    {
        if (!this.isInitialized())
            return;
        if (usesTalonPositionControl(mDriveControlState))
        {
            mMotorGroup.drivePositionInches(leftPosInches, rightPosInches);
        }
        else
        {
            logError("Hit a bad position control state");
            mMotorGroup.stop();
        }
    }

    /**
     * Update the heading at which the robot thinks the boiler is.
     *
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateGoalHeading(double timestamp)
    {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        if (aim.isPresent())
        {
            mTargetHeading = aim.get().getRobotToGoal();
        }
    }

    /**
     * Turn the robot to a target heading.
     *
     * Is called periodically when the robot is auto-aiming towards a target.
     */
    private void updateTurnToFieldHeading(double timestamp)
    {
        if (!this.isInitialized())
            return;
        // We need the field frame because mTargetHeading is specified in field coordinates, not robot ones
        final Rotation2d fieldToRobot =
                mRobotState.getLatestFieldToVehicle().getValue().getRotation();
        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robotToTarget = fieldToRobot.inverse().rotateBy(mTargetHeading);

        performClosedLoopTurn(robotToTarget);
    }

    // I'm going to switch this up, and the name may be a bit strange for its job
    // TODO: Fix the name
    private void updateTurnToRobotHeading(double timestamp, double heading)
    {
        if (!this.isInitialized())
            return;
        final Rotation2d robotToTarget = Rotation2d.fromDegrees(heading);
        performClosedLoopTurn(robotToTarget);
        // The angle might not need be reversed. This was probabbly from old data.
        // angle reversed to correct for raspi coordsys
    }

    private void searchForCube(double timestamp)
    {
        if (!this.isInitialized())
            return;
        double dx = mVisionTargetAngleEntry.getNumber(0).doubleValue();
        if (!Util.epsilonEquals(dx, 0, 30))
        {
            // If our target is within reasonable bounds
            dx = 17;
        }
        updateTurnToRobotHeading(timestamp, dx);

    }

    /*
     * Trigger updates to closed-loop position based on a rotation expressed
     * relative to the robot's current heading.
     * This actually doesn't use the IMU... Only the wheel encoders and
     * kinematics.
     */
    private void performClosedLoopTurn(Rotation2d robotToTarget)
    {
        // Check if we are on target
        final double kGoalPosTolerance = 1; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robotToTarget.getDegrees()) < kGoalPosTolerance
                && Math.abs(mMotorGroup.getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(mMotorGroup.getRightVelocityInchesPerSec()) < kGoalVelTolerance)
        {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
        }
        else
        {
            Kinematics.DriveVelocity wheel_delta = Kinematics
                    .inverseKinematics(new Twist2d(0, 0, robotToTarget.getRadians()));
            updatePositionSetpoint(wheel_delta.left + mMotorGroup.getLeftDistanceInches(),
                    wheel_delta.right + mMotorGroup.getRightDistanceInches());
        }
    }

    /**
     * Essentially does the same thing as updateTurnToHeading but sends the
     * robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */
    private void updateDriveTowardsGoalCoarseAlign(double timestamp)
    {
        if (!this.isInitialized())
            return;
        updateGoalHeading(timestamp);
        updateTurnToFieldHeading(timestamp);
        mIsApproaching = true;
        if (mIsOnTarget)
        {
            // Done coarse alignment.

            Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
            if (aim.isPresent())
            {
                final double distance = aim.get().getRange();

                if (distance < Constants.kShooterOptimalRangeCeiling &&
                        distance > Constants.kShooterOptimalRangeFloor)
                {
                    // Don't drive, just shoot.
                    mDriveControlState = DriveControlState.AIM_TO_GOAL;
                    mIsApproaching = false;
                    mIsOnTarget = false;
                    updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                            mMotorGroup.getRightDistanceInches());
                    return;
                }
            }

            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
            mIsOnTarget = false;
        }
    }

    /**
     * Drives the robot straight forwards until it is at an optimal shooting
     * distance. Then sends the robot into the AIM_TO_GOAL state for one final
     * alignment
     */
    private void updateDriveTowardsGoalApproach(double timestamp)
    {
        if (!this.isInitialized())
            return;
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        mIsApproaching = true;
        if (aim.isPresent())
        {
            final double distance = aim.get().getRange();
            double error = 0.0;
            if (distance < Constants.kShooterOptimalRangeFloor)
            {
                error = distance - Constants.kShooterOptimalRangeFloor;
            }
            else if (distance > Constants.kShooterOptimalRangeCeiling)
            {
                error = distance - Constants.kShooterOptimalRangeCeiling;
            }
            final double kGoalPosTolerance = 1.0; // inches
            if (Util.epsilonEquals(error, 0.0, kGoalPosTolerance))
            {
                // We are on target. Switch back to auto-aim.
                mDriveControlState = DriveControlState.AIM_TO_GOAL;
                RobotState.getInstance().resetVision();
                mIsApproaching = false;
                updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                        mMotorGroup.getRightDistanceInches());
                return;
            }
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches() + error,
                    mMotorGroup.getRightDistanceInches() + error);
        }
        else
        {
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
        }
    }

    /**
     * Called periodically when the robot is in path following mode. Updates the
     * path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity
     * setpoints.
     */
    private void updatePathFollower(double timestamp)
    {
        if (!this.isInitialized())
            return;
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(),
                RobotState.getInstance().getPredictedVelocity().dx);

        if (!mPathFollower.isFinished())
        {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        }
        else
        {
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isOnTarget()
    {
        // return true;
        return mIsOnTarget;
    }

    public synchronized boolean isAutoAiming()
    {
        return mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    // public synchronized void setWantPositionControl()
    // public synchronized void setWantVelocityControl()
    //  these methods are already implemented above but with an inconsistent naming
    //  see: setVelocitySetpoint, setPositionSetpoint, setRelativePosition, setOpenLoop

    /**
     * Configures the drivebase for auto aiming
     */
    public synchronized void setWantAimToGoal()
    {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL)
        {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(mMotorGroup.getGyroAngle());
        }
    }

    public synchronized void setWantAimToVisionTarget()
    {
        if (mDriveControlState != DriveControlState.TURN_TO_ROBOTANGLE)
        {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.TURN_TO_ROBOTANGLE;
            // no target received yet, start at current heading
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(mMotorGroup.getGyroAngle());
        }
    }

    public synchronized void setWantSearchForCube()
    {
        if (mDriveControlState != DriveControlState.FIND_CUBE)
        {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.FIND_CUBE;
            // no target received yet, start at current heading
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(mMotorGroup.getGyroAngle());
        }
    }

    /**
     * Configures the drivebase for auto driving
     */
    public synchronized void setWantDriveTowardsGoal()
    {
        if (mDriveControlState != DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN &&
                mDriveControlState != DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH &&
                mDriveControlState != DriveControlState.AIM_TO_GOAL)
        {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN;
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(mMotorGroup.getGyroAngle());
        }
    }

    /**
     * Configures the drivebase to turn to a desired heading
     */
    public synchronized void setWantTurnToHeading(Rotation2d heading)
    {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING)
        {
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
            updatePositionSetpoint(mMotorGroup.getLeftDistanceInches(),
                    mMotorGroup.getRightDistanceInches());
        }
        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3)
        {
            mTargetHeading = heading;
            mIsOnTarget = false;
        }
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed)
    {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING)
        {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv,
                            Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance,
                            Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        }
        else
        {
            setVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isDoneWithPath()
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
        {
            return mPathFollower.isFinished();
        }
        else
        {
            logError("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath()
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
        {
            mPathFollower.forceFinish();
        }
        else
        {
            logError("Robot is not in path following mode");
        }
    }

    public boolean isApproaching()
    {
        return mIsApproaching;
    }

    public synchronized boolean isDoneWithTurn()
    {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING)
        {
            return mIsOnTarget;
        }
        else
        {
            logError("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized boolean hasPassedMarker(String marker)
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
        {
            return mPathFollower.hasPassedMarker(marker);
        }
        else
        {
            logError("Robot is not in path following mode");
            return false;
        }
    }

    // fill our two slots with gains...
    public synchronized void reloadGains()
    {
        if (mMotorGroup.isInitialized())
        {
            mMotorGroup.reloadGains(kPositionControlSlot,
                    Constants.kDrivePositionKp, 
                    Constants.kDrivePositionKi,
                    Constants.kDrivePositionKd, 
                    Constants.kDrivePositionKf, // left
                    Constants.kDrivePositionKf, // right
                    Constants.kDrivePositionIZone, 
                    Constants.kDrivePositionMaxIAccum,
                    Constants.kDrivePositionRampRate);

            mMotorGroup.reloadGains(kVelocityControlSlot,
                    Constants.kDriveVelocityKp, 
                    Constants.kDriveVelocityKi,
                    Constants.kDriveVelocityKd,
                    Constants.kDriveVelocityKf, // left 
                    Constants.kDriveVelocityKf, // right 
                    Constants.kDriveVelocityIZone, 
                    Constants.kDriveVelocityMaxIAccum,
                    Constants.kDriveVelocityRampRate);
            
            // old Kf notes (when we'd forgotten to set up correct initial pose):
            //      +1.05 on right at 10 ips is pretty good on the 2nd robot, 
            //      f of 0.2 on left and 0 on right is great for 2nd robot!

            logNotice("reloaded PID gains");
            logDebug("reloaded position gains:" + mMotorGroup.dumpPIDState(kPositionControlSlot));
            logDebug("reloaded velocity gains:" + mMotorGroup.dumpPIDState(kVelocityControlSlot));
            // nb: motionMagic velocity and accel aren't slot-based, so should be established
            //   when we enter the control mode.
        }
    }

    @Override
    public void writeToLog()
    {
        mCSVWriter.write();
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state)
    {
        if (state == DriveControlState.VELOCITY_SETPOINT
                || state == DriveControlState.PATH_FOLLOWING)
        {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state)
    {
        if (state == DriveControlState.POSITION_SETPOINT ||
                state == DriveControlState.AIM_TO_GOAL ||
                state == DriveControlState.TURN_TO_ROBOTANGLE ||
                state == DriveControlState.TURN_TO_HEADING ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH ||
                state == DriveControlState.FIND_CUBE)
        {
            return true;
        }
        return false;
    }

    @Override
    public boolean checkSystem(String variant)
    {
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        logNotice("checkSystem " + variant + "  ----------------");
        boolean success = true;
        if (variant == "velocityControl")
        {
            Timer timer = new Timer();
            logNotice("  enter velocity control mode");
            configureTalonsForSpeedControl();

            logNotice("  straight: 10ips, 4 sec");
            this.setVelocitySetpoint(10, 10);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }
            this.setVelocitySetpoint(0, 0);
            this.outputToSmartDashboard();
            this.stop();

            Timer.delay(2);
            logNotice("  straight: -10ips, 4 sec");
            this.setVelocitySetpoint(-10, -10);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }
            this.setVelocitySetpoint(0, 0);
            this.outputToSmartDashboard();
            this.stop();

            Timer.delay(2);
            logNotice("  curve left: 4 sec");
            this.setVelocitySetpoint(3, 5);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }
            this.setVelocitySetpoint(0, 0);
            this.outputToSmartDashboard();
            this.stop();

            Timer.delay(2);
            logNotice("  curve right: 4 sec");
            this.setVelocitySetpoint(5, 3);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }
            this.setVelocitySetpoint(0, 0);
            this.outputToSmartDashboard();
            this.stop();

            Timer.delay(2);
            logNotice("  mixed speeds straight: 10, 4, 20, 4");

            this.setVelocitySetpoint(10, 10);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }

            this.setVelocitySetpoint(4, 4);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }

            timer.reset();
            timer.start();
            this.setVelocitySetpoint(20, 20);
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }

            this.setVelocitySetpoint(4, 4);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputToSmartDashboard();
            }
            this.stop();
            this.setVelocitySetpoint(0, 0);
            this.outputToSmartDashboard();
        }
        else
            success = mMotorGroup.checkSystem(variant);
        return success;
    }

    public DriveControlState getState()
    {
        // Get drive train state
        return mDriveControlState;
    }

    public boolean onVisionTarget()
    {
        // In a perfect world this number is usable to all of drive
        final double dx = mVisionTargetAngleEntry.getNumber(0).doubleValue();
        if (Util.epsilonEquals(dx, 0, 1))
        {
            // We are on target
            return true;
        }
        else
        {
            // We are not on target
            return false;
        }
    }
}
