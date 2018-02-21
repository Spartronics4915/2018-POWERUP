package com.spartronics4915.frc2018;

import com.spartronics4915.lib.util.ConstantsBase;
import com.spartronics4915.lib.util.math.Translation2d;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * A list of constants used by the rest of the robot code. 
 * File separates hardware configuration constants from software
 * constants.  The software constants include various timeout periods
 * as well as physics constants and those determined through calibrations.
 * 
 * Constant names should include the subsystem or field element (etc)
 * to which they refer.
 */
public class Constants extends ConstantsBase
{
    public static final boolean kUseTestbedConstants = false; // XXX: We should use the ConstantsBase reflection -> file instead of this
    // Hardware configuration constants --------------------------------------------------------
    
    // CAN Bus --------------------------- 
    // there are at least three families of ids on the CAN bus defined by
    // these device classes:
    //      PDP - power distribution panel (only one of these)
    //      SRX - CANTalon Ids
    //      PCM - Pressure Control Module Ids (often 0 or 1)
    
    //  -- Talon SRX Channels --------------
    //      (Note that if multiple talons are dedicated to a mechanism, any sensors
    //      are attached to the master)
    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveSlaveId = 2;
    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriveSlaveId = 4;
    public static final int kDriveIMUTalonId = kRightDriveSlaveId; // must be a slave
    
    public static final int kClimberWinchPrimaryMotorId = 5;
    public static final int kGrabberFlipperMotorId = 7;
    public static final int kHarvesterLeftMotorId = kUseTestbedConstants ? 18 : 8;
    public static final int kHarvesterRightMotorId = kUseTestbedConstants ? 16 : 9;
 
    public static final int kNumTalons = 8; // total talon count on robot (not testbed)
    
    public static final int kNumPDPs = 1; // doesn't always show up in CANProbe
    public static final int kNumPCMs = 1; // Pressure control module (pneumatics)
    public static final int kNumCANDevices = kNumTalons + kNumPCMs; // don't count PDP
    
    // -- Pressure Control Module (PCM) Channels ----   
    public static final int kScissorUpSolenoidId = 0; //PCM 0
    public static final int kScissorDownSolenoidId = 1; //PCM 0
    public static final int kScissorBrakeSolenoidId = 2; //PCM 0
    public static final int kGrabberSetupSolenoidId = 3; //PCM 0
    public static final int kGrabberSolenoidId = 4; //PCM 0
    public static final int kHarvesterSolenoidId = 5; //PCM 0
    public static final int kClimberStabilizationSolenoidId1 = 6; //PCM 0
    public static final int kClimberStabilizationSolenoidId2 = 7; //PCM 0

    // PWM (Servo) Pins ----------------------------
    
    // Relay Pins -----------------------------------
    public static final int kLEDVisionLampId = 0;
    public static final int kLEDDriverLEDId = 1;
    
    // DIO Pins --------------------------------------
    public static final int kFlipperRevLimitSwitchId = 0;
    public static final int kFlipperFwdLimitSwitchId = 2;
    
    // Analog In Pins ---------------------------------
    public static final int kScissorHeightPotentiometerId = 0;
    public static final int kGrabberAnglePotentiometerId = 1;
    public static final int kGrabberCubeDistanceRangeFinderId = 2;

    
    // Software configuration constants ----------------------------------------------------------
    public static final double kLooperDt = 0.005;
    
    // Vision
    public static final int kAndroidAppTcpPort = 8254;

    /* ROBOT PHYSICAL CONSTANTS ---------------------------------------------------------  */

    // Wheels
    public static final double kDriveWheelDiameterInches = 6;
    public static final double kTrackWidthInches = 23.75;
    public static final double kTrackScrubFactor = 0.624;

    // Chassis Geometry -- Currently unused
    public static final double kCenterToFrontBumperDistance = 17.839285714;
    // public static final double kCenterToIntakeDistance = 20.9675;
    public static final double kCenterToRearBumperDistance = kCenterToFrontBumperDistance;
    public static final double kCenterToSideBumperDistance = 15.375;

    /* CONTROL LOOP GAINS ---------------------------------------------------------------- */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static final double kDriveVelocityKp = 0.2;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 150;
    public static final double kDriveVelocityKf = .25;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVelocityRampRate = .05; // 240V/s -> 12V in .05s
    public static final double kDriveHighGearNominalOutput = 0.5;
    public static final double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive position loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static final double kDrivePositionKp = 0.85;
    public static final double kDrivePositionKi = 0.002;
    public static final double kDrivePositionKd = 100.0;
    public static final double kDrivePositionKf = .45;
    public static final int kDrivePositionIZone = 700;
    public static final double kDrivePositionRampRate = .25; // 48.0 V/s -> 12V in .25s
    public static final double kDriveLowGearNominalOutput = 0.5; // pct
    public static final double kDriveLowGearMaxVelocity = 3.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
                                                                                                               // in RPM
    public static final double kDriveLowGearMaxAccel = 15.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 15 fps/s
                                                                                                             // in RPM/s
    public static final double kDriveVoltageCompensationRampRate = 0.0;
    
    
    // Drive ------------------------------------------------------
    // Encoder: https://cdn.usdigital.com/assets/datasheets/E4P_datasheet.pdf?k=636523267170919858
    // modelr: E4P-360-250-N-S-D-D (western digital)
    //              ^ EncoderCodesPerRev (later multiplied by 4 for quad)
    //                  ^ .25" shaft mount 
    //                     ^ "No Index"
    //                        ^ "Single Output" (not Differential)
    //                          ^ ^ Default cover, Default base
    public static final int kEncoderCodesPerRev = 360;


    // Path following constants -----------------------------------
    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 9.0; // inches per second
    public static final double kMaxLookAhead = 24.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static final double kSegmentCompletionTolerance = 0.05; // inches
    public static final double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    public static final double kPathFollowingProfileKp = 20.00;
    public static final double kPathFollowingProfileKi = 0.08;
    public static final double kPathFollowingProfileKv = 0.02;
    public static final double kPathFollowingProfileKffv = 1.0;
    public static final double kPathFollowingProfileKffa = 0.05;
    public static final double kPathFollowingGoalPosTolerance = 0.75;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 9.0;

    // Goal tracker constants
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 30.0;
    public static final double kTrackReportComparatorStablityWeight = 1.0;
    public static final double kTrackReportComparatorAgeWeight = 1.0;

    // Pose of the camera frame w.r.t. the robot frame
    public static final String kVisionTableName = "Vision"; // name in networktables below root
    public static final String kVisionTargetAngleName = "ax"; // "clock", "ay" are also available
    public static final double kCameraXOffset = -3.3211;
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraZOffset = 20.9;
    public static final double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraDeadband = 0.0;

    public static final double kShooterOptimalRange = 100.0;
    public static final double kShooterOptimalRangeFloor = 95.0;
    public static final double kShooterOptimalRangeCeiling = 105.0;

    public static final double kShooterAbsoluteRangeFloor = 90.0;
    public static final double kShooterAbsoluteRangeCeiling = 130.0;
    
    // Field constants
    public static final double kFieldWidth = 648;
    public static final double kFieldHeight = 324;
    public static final Translation2d kFieldDimensionTranslation = new Translation2d(kFieldWidth, kFieldHeight);
    
    /**
     * Make an {@link Solenoid} instance for the single-number ID of the
     * solenoid
     * 
     * @param solenoidId
     *        One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId)
    {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
    }

    @Override
    public String getFileLocation()
    {
        return "~/constants.txt";
    }
}
