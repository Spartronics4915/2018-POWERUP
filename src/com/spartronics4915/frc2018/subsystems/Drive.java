package com.spartronics4915.frc2018.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.Kinematics;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.ShooterAimingParameters;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.control.Lookahead;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.control.PathFollower;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Twist2d;
import com.spartronics4915.lib.util.drivers.CANTalonFactory;
import com.spartronics4915.lib.util.drivers.CANTalonPhoenix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons,
 * one solenoid and 2 pistons to shift gears,
 * and a Pigeon IMU board. The Drive subsystem has several control methods
 * including
 * open loop, velocity control, and position
 * control. The Drive subsystem also has several methods that handle automatic
 * aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem
{

    private static Drive mInstance = null;

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

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
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // turn to face the boiler
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
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
        if (state == DriveControlState.AIM_TO_GOAL ||
                state == DriveControlState.TURN_TO_HEADING ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH)
        {
            return true;
        }
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private CANTalonPhoenix mLeftMaster = null, mRightMaster = null;
    private CANTalonPhoenix mLeftSlave = null, mRightSlave = null;
    private CANTalonPhoenix mIMUTalon = null;
    private PigeonIMU mIMU = null;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;

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
                 setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                setVelocitySetpoint(0, 0);
                if (mIMU.getState() != PigeonState.Ready)
                {
                    DriverStation.reportError("IMU in non-ready state. Is it plugged in?", false);
                    return;
                }
                mIMU.setYaw(0, 5); // was SetYaw(0)
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
                        return;
                    case VELOCITY_SETPOINT:
                        return;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null)
                        {
                            updatePathFollower(timestamp);
                            mCSVWriter.add(mPathFollower.getDebug());
                        }
                        return;
                    case TURN_TO_HEADING:
                        updateTurnToHeading(timestamp);
                        return;
                    case DRIVE_TOWARDS_GOAL_COARSE_ALIGN:
                        updateDriveTowardsGoalCoarseAlign(timestamp);
                        return;
                    case DRIVE_TOWARDS_GOAL_APPROACH:
                        updateDriveTowardsGoalApproach(timestamp);
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
        // Start all Talons in open loop mode.
        CANProbe canProbe = CANProbe.getInstance();

        if (!canProbe.validateSRXId(Constants.kLeftDriveMasterId))
        {
            logError("Can't find left master motor");
        }
        else
        {
            mLeftMaster = CANTalonFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
            mLeftMaster.changeControlMode(ControlMode.PercentOutput); // XXX: was PercentVBus
            mLeftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
            mLeftMaster.configEncoderCodesPerRev(Constants.kEncoderCodesPerRev);
            mLeftMaster.reverseSensor(false); // If these aren't correctly reversed your PID will just spiral out of control
            mLeftMaster.reverseOutput(false);
            if (!mLeftMaster.isSensorPresent(FeedbackDevice.QuadEncoder))
            {
                logError("Could not detect left encoder");
            }
            if (!canProbe.validateSRXId(Constants.kLeftDriveSlaveId))
            {
                logError("Can't find left slave motor");
            }
            else
            {
                mLeftSlave = CANTalonFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveId,
                        Constants.kLeftDriveMasterId);
                mLeftSlave.reverseOutput(false);
            }
            mLeftMaster.setStatusFrameRateMs(StatusFrameEnhanced.Status_2_Feedback0, 5); // XXX: was Feedback
        }

        if (!canProbe.validateSRXId(Constants.kRightDriveMasterId))
        {
            logError("Can't find right master motor");
        }
        else
        {
            mRightMaster = CANTalonFactory.createDefaultTalon(Constants.kRightDriveMasterId);
            mRightMaster.changeControlMode(ControlMode.PercentOutput); // XXX: was PercentVBus
            mRightMaster.reverseSensor(true);
            mRightMaster.reverseOutput(true);
            mRightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
            mRightMaster.configEncoderCodesPerRev(Constants.kEncoderCodesPerRev);
            if (!mRightMaster.isSensorPresent(FeedbackDevice.QuadEncoder))
            {
                logError("Could not detect right encoder");
            }

            if (!canProbe.validateSRXId(Constants.kRightDriveSlaveId))
            {
                logError("Can't find right master motor");
            }
            else
            {
                mRightSlave =
                        CANTalonFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveId,
                                Constants.kRightDriveMasterId);
                mRightSlave.reverseOutput(false);
                mRightMaster.setStatusFrameRateMs(StatusFrameEnhanced.Status_2_Feedback0, 5); // XXX: was Feedback
            }
        }

        if (mLeftMaster != null)
        {
            mLeftMaster.setVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
            mLeftMaster.setVelocityMeasurementWindow(32);
        }
        if (mRightMaster != null)
        {
            mRightMaster.setVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
            mRightMaster.setVelocityMeasurementWindow(32);
        }

        if (mRightMaster != null && mRightSlave != null &&
                mLeftMaster != null && mLeftSlave != null)
        {
            reloadGains();
            mIsHighGear = false;
            setHighGear(true);
            setOpenLoop(DriveSignal.NEUTRAL);
            // Path Following stuff
            mIMUTalon = new CANTalonPhoenix(Constants.kIMUTalonId);
            // FIXME: Don't use the pigeon, or at least wire it directly into the CAN bus
            mIMU = new PigeonIMU(mIMUTalon);

            if (mIMU.getState() == PigeonState.NoComm)
                logError("Could not detect the IMU. Is it plugged in?");

            // Force a CAN message across.
            mIsBrakeMode = true;
            setBrakeMode(false);
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
        if(!Drive.this.isInitialized()) return;

        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal)
    {
        if(!this.isInitialized()) return;
        if (mDriveControlState != DriveControlState.OPEN_LOOP)
        {
            mLeftMaster.changeControlMode(ControlMode.PercentOutput); // XXX: was PctVBus
            mRightMaster.changeControlMode(ControlMode.PercentOutput);
            mLeftMaster.configNominalOutputVoltage(0.0, 0.0);
            mRightMaster.configNominalOutputVoltage(0.0, 0.0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        logInfo("setOpenLoop:" + signal.getLeft());
        mRightMaster.set(signal.getRight());
        mLeftMaster.set(signal.getLeft());
    }

    public boolean isHighGear()
    {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear)
    {
        // XXX: Dead code
        if (wantsHighGear != mIsHighGear)
        {
            mIsHighGear = wantsHighGear;
        }
    }

    public boolean isBrakeMode()
    {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on)
    {
        if(!this.isInitialized()) return;
        if (mIsBrakeMode != on)
        {
            mIsBrakeMode = on;
            mRightMaster.enableBrakeMode(on);
            mRightSlave.enableBrakeMode(on);
            mLeftMaster.enableBrakeMode(on);
            mLeftSlave.enableBrakeMode(on);
        }
    }

    @Override
    public synchronized void stop()
    {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard()
    {
        if(!this.isInitialized()) return;
        final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
        SmartDashboard.putNumber("left voltage (V)", mLeftMaster.getOutputVoltage());
        SmartDashboard.putNumber("right voltage (V)", mRightMaster.getOutputVoltage());
        SmartDashboard.putNumber("left speed (ips)", left_speed);
        SmartDashboard.putNumber("right speed (ips)", right_speed);
        if (usesTalonVelocityControl(mDriveControlState))
        {
            SmartDashboard.putNumber("left speed error (ips)",
                    rpmToInchesPerSecond(mLeftMaster.getSetpoint()) - left_speed);
            SmartDashboard.putNumber("right speed error (ips)",
                    rpmToInchesPerSecond(mRightMaster.getSetpoint()) - right_speed);
        }
        else
        {
            SmartDashboard.putNumber("left speed error (ips)", 0.0);
            SmartDashboard.putNumber("right speed error (ips)", 0.0);
        }
        synchronized (this)
        {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
            {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            }
            else
            {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
        SmartDashboard.putNumber("Drivetrain_IMU_Heading", getGyroAngle().getDegrees());
        SmartDashboard.putBoolean("drive on target", isOnTarget());
    }

    public synchronized void resetEncoders()
    {
        if(!this.isInitialized()) return;
        mLeftMaster.setEncPosition(0);
        mLeftMaster.setPosition(0);
        mRightMaster.setPosition(0);
        mRightMaster.setEncPosition(0);
        mLeftSlave.setPosition(0);
        mRightSlave.setPosition(0);
    }

    @Override
    public void zeroSensors()
    {
        if(!this.isInitialized()) return;
        resetEncoders();
        if (mIMU.getState() != PigeonState.Ready)
        {
            DriverStation.reportError("IMU in non-ready state. Is it plugged in?", false);
            return;
        }
        mIMU.setYaw(0, 5/* timeoutMS */);
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec,
            double right_inches_per_sec)
    {
        if(!this.isInitialized()) return;
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl()
    {
        if(!this.isInitialized()) return;
        if (!usesTalonVelocityControl(mDriveControlState))
        {
            // We entered a velocity control state.
            mLeftMaster.changeControlMode(ControlMode.Velocity); // XXX: was speed
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(kHighGearVelocityControlSlot);
            mLeftMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
                    -Constants.kDriveHighGearNominalOutput);
            mRightMaster.changeControlMode(ControlMode.Velocity); // XXX: was speed
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(kHighGearVelocityControlSlot);
            mRightMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
                    -Constants.kDriveHighGearNominalOutput);
            setBrakeMode(true);
        }
    }

    /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl()
    {
        if(!this.isInitialized()) return;
        if (!usesTalonPositionControl(mDriveControlState))
        {
            // We entered a position control state.
            mLeftMaster.changeControlMode(ControlMode.MotionMagic);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(kLowGearPositionControlSlot);
            mLeftMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            mRightMaster.changeControlMode(ControlMode.MotionMagic);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(kLowGearPositionControlSlot);
            mRightMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            setBrakeMode(true);
        }
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec,
            double right_inches_per_sec)
    {
        if(!this.isInitialized()) return;
        if (usesTalonVelocityControl(mDriveControlState))
        {
            final double max_desired =
                    Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            mLeftMaster.set(inchesPerSecondToRpm(left_inches_per_sec * scale));
            mRightMaster.set(inchesPerSecondToRpm(right_inches_per_sec * scale));
        }
        else
        {
            logError("Hit a bad velocity control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
    }

    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updatePositionSetpoint(double left_position_inches,
            double right_position_inches)
    {
        if(!this.isInitialized()) return;
        if (usesTalonPositionControl(mDriveControlState))
        {
            mLeftMaster.set(inchesToRotations(left_position_inches));
            mRightMaster.set(inchesToRotations(right_position_inches));
        }
        else
        {
            logError("Hit a bad position control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
    }

    private static double rotationsToInches(double rotations)
    {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm)
    {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches)
    {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second)
    {
        return inchesToRotations(inches_per_second) * 60;
    }

    public double getLeftDistanceInches()
    {
        if(!isInitialized()) return 0.0;
        return rotationsToInches(mLeftMaster.getPosition());
    }

    public double getRightDistanceInches()
    {
        if(!isInitialized()) return 0.0;
        return rotationsToInches(mRightMaster.getPosition());
    }

    public double getLeftVelocityInchesPerSec()
    {
        if(!isInitialized()) return 0.0;
        return rpmToInchesPerSecond(mLeftMaster.getSpeed());
    }

    public double getRightVelocityInchesPerSec()
    {
        if(!isInitialized()) return 0.0;
        return rpmToInchesPerSecond(mRightMaster.getSpeed());
    }

    public synchronized Rotation2d getGyroAngle()
    {
        if(!this.isInitialized()) new Rotation2d();
        if (mIMU.getState() != PigeonState.Ready)
        {
            DriverStation.reportError("IMU in non-ready state. Is it plugged in?", false);
            return Rotation2d.fromDegrees(0);
        }
        double[] ypr = new double[3]; // This is ridiculous. Quick fix: don't use the pigeon!
        mIMU.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]); // Rotation2d normalizes between -180 and 180 automatically
    }

    public synchronized void setGyroAngle(Rotation2d angle)
    {
        if(!this.isInitialized()) return;
        if (mIMU.getState() == PigeonState.NoComm)
            DriverStation.reportError("Could not detect the IMU. Is it plugged in?", false);
        mIMU.setYaw(angle.getDegrees(), 5 /* delayMS */);
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
     * Is called periodically when the robot is auto-aiming towards the boiler.
     * 
     * This actually doesn't use the IMU... Only the wheel encoders and
     * kinematics.
     */
    private void updateTurnToHeading(double timestamp)
    {
        if(!this.isInitialized()) return;
        final Rotation2d field_to_robot =
                mRobotState.getLatestFieldToVehicle().getValue().getRotation(); // We need the field frame because this is specified in field coordinates, not robot ones
        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        // Check if we are on target
        final double kGoalPosTolerance = 0.75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance)
        {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics
                .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
                wheel_delta.right + getRightDistanceInches());
    }

    /**
     * Essentially does the same thing as updateTurnToHeading but sends the
     * robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */
    private void updateDriveTowardsGoalCoarseAlign(double timestamp)
    {
        if(!this.isInitialized()) return;
        updateGoalHeading(timestamp);
        updateTurnToHeading(timestamp);
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
                    updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                    return;
                }
            }

            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
            mIsOnTarget = false;
        }
    }

    /**
     * Drives the robot straight forwards until it is at an optimal shooting
     * distance. Then sends the robot into the
     * AIM_TO_GOAL state for one final alignment
     */
    private void updateDriveTowardsGoalApproach(double timestamp)
    {
        if(!this.isInitialized()) return;
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
                updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                return;
            }
            updatePositionSetpoint(getLeftDistanceInches() + error,
                    getRightDistanceInches() + error);
        }
        else
        {
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
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
        if(!this.isInitialized()) return;
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
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            mTargetHeading = getGyroAngle();
        }
        setHighGear(false);
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
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            mTargetHeading = getGyroAngle();
        }
        setHighGear(false);
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
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
        }
        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3)
        {
            mTargetHeading = heading;
            mIsOnTarget = false;
        }
        setHighGear(false);
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

    public synchronized void reloadGains()
    {
        if(!isInitialized()) return;
        mLeftMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        mLeftMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity); // TODO
        mLeftMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);
        mRightMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        mRightMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity); // TODO
        mRightMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);
        mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
        mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);

        mLeftMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        mRightMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
        mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
    }

    @Override
    public void writeToLog()
    {
        mCSVWriter.write();
    }

    public boolean checkSystem()
    {
        if(!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        logNotice("checkSystem() ---------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 300;
        final double kMaxVoltage = 12.0;

        mRightMaster.changeControlMode(ControlMode.PercentOutput); // was Voltage
        mRightSlave.changeControlMode(ControlMode.PercentOutput);
        mLeftMaster.changeControlMode(ControlMode.PercentOutput);
        mLeftSlave.changeControlMode(ControlMode.PercentOutput);

        mRightMaster.set(0.0);
        mRightSlave.set(0.0);
        mLeftMaster.set(0.0);
        mLeftSlave.set(0.0);

        mRightMaster.set(-6.0f / kMaxVoltage);
        Timer.delay(4.0);
        final double currentRightMaster = mRightMaster.getOutputCurrent();
        final double rpmRightMaster = mRightMaster.getSpeed();
        mRightMaster.set(0.0f);

        Timer.delay(2.0);

        mRightSlave.set(-6.0f / kMaxVoltage);
        Timer.delay(4.0);
        final double currentRightSlave = mRightSlave.getOutputCurrent();
        final double rpmRightSlave = mRightMaster.getSpeed();
        mRightSlave.set(0.0f);

        Timer.delay(2.0);

        mLeftMaster.set(6.0f / kMaxVoltage);
        Timer.delay(4.0);
        final double currentLeftMaster = mLeftMaster.getOutputCurrent();
        final double rpmLeftMaster = mLeftMaster.getSpeed();
        mLeftMaster.set(0.0f);

        Timer.delay(2.0);

        mLeftSlave.set(6.0f / kMaxVoltage);
        Timer.delay(4.0);
        final double currentLeftSlave = mLeftSlave.getOutputCurrent();
        final double rpmLeftSlave = mLeftMaster.getSpeed();
        mLeftSlave.set(0.0);

        mRightMaster.changeControlMode(ControlMode.PercentOutput);
        mLeftMaster.changeControlMode(ControlMode.PercentOutput);

        mRightSlave.changeControlMode(ControlMode.Follower);
        mRightSlave.set(Constants.kRightDriveMasterId);

        mLeftSlave.changeControlMode(ControlMode.Follower);
        mLeftSlave.set(Constants.kLeftDriveMasterId);

        logNotice("Right Master Current: " + currentRightMaster + " Drive Right Slave Current: "
                + currentRightSlave);
        logNotice("Left Master Current: " + currentLeftMaster + " Drive Left Slave Current: "
                + currentLeftSlave);
        logNotice("RPM RMaster: " + rpmRightMaster + " RSlave: " + rpmRightSlave + " LMaster: "
                + rpmLeftMaster + " LSlave: " + rpmLeftSlave);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Right Master Current Low !!!!!!!!!!");
        }

        if (currentRightSlave < kCurrentThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Right Slave Current Low !!!!!!!!!!");
        }

        if (currentLeftMaster < kCurrentThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Left Master Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave < kCurrentThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Left Slave Current Low !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave),
                currentRightMaster,
                5.0))
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Right Currents Different !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentLeftMaster, currentLeftSlave), currentLeftSlave,
                5.0))
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Drive Left Currents Different !!!!!!!!!!!!!");
        }

        if (rpmRightMaster < kRpmThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Drive Right Master RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmRightSlave < kRpmThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Drive Right Slave RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftMaster < kRpmThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Drive Left Master RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftSlave < kRpmThres)
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!! Drive Left Slave RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(
                Arrays.asList(rpmRightMaster, rpmRightSlave, rpmLeftMaster, rpmLeftSlave),
                rpmRightMaster, 250))
        {
            failure = true;
            logWarning("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }
}
