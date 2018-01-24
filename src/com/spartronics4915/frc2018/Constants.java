package com.spartronics4915.frc2018;

import com.spartronics4915.lib.util.ConstantsBase;
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
    
    public static final int kTestbedMotor1Id = 16;
    public static final int kTestbedMotor2Id = 18;
 
    public static final int kNumTalons = 4; // total talon count on robot (not testbed)
    
    public static final int kNumPDPs = 0; // CANProbe doesn't probe for this device
    public static final int kNumPCMs = 1; // Pressure control module (pneumatics)
    public static final int kNumCANDevices = kNumTalons + kNumPCMs + kNumPDPs;
    
    // -- Pressure Control Module (PCM) Channels ----
    public static final int kShifterSolenoidId = 0; // PCM 0, Solenoid 0
    public static final int kIntakeDeploySolenoidId = 1; // PCM 0, Solenoid 1
    public static final int kHopperSolenoidId = 2; // PCM 0, Solenoid 2
    public static final int kGearWristSolenoid = 7; // PCM 0, Solenoid 7    

    // PWM (Servo) Pins ----------------------------
    public static final int kTestbedServoId = 0;
    
    // Relay Pins -----------------------------------
    public static final int kTestbedLightSwitchId = 0;
    
    // DIO Pins --------------------------------------
    public static final int kTestbedLimitSwitchId = 0;
    public static final int kRangeLEDId = 8;
    public static final int kGreenLEDId = 9;
    
    // Analog In Pins ---------------------------------
    public static final int kTestbedPotentiometerId = 0;
    public static final int kLEDOnId = 2;
    
    // Software configuration constants ----------------------------------------------------------

    public static final double kLooperDt = 0.005;
    
    // Vision
    public static final int kAndroidAppTcpPort = 8254;

    /* ROBOT PHYSICAL CONSTANTS ---------------------------------------------------------  */

    // Wheels
    public static final double kDriveWheelDiameterInches = 6;
    public static final double kTrackWidthInches = 23.75;
    public static final double kTrackScrubFactor = 0.624;

    // Chassis Geometry
    public static final double kCenterToFrontBumperDistance = 16.14;
    // public static final double kCenterToIntakeDistance = 20.9675;
    public static final double kCenterToRearBumperDistance = 16.14;
    public static final double kCenterToSideBumperDistance = 13.78;

    /* CONTROL LOOP GAINS ---------------------------------------------------------------- */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static final double kDriveHighGearVelocityKp = 1.2;
    public static final double kDriveHighGearVelocityKi = 0.0;
    public static final double kDriveHighGearVelocityKd = 6.0;
    public static final double kDriveHighGearVelocityKf = .15;
    public static final int kDriveHighGearVelocityIZone = 0;
    public static final double kDriveHighGearVelocityRampRate = .05; // 240V/s -> 12V in .05s
    public static final double kDriveHighGearNominalOutput = 0.5;
    public static final double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive position loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static final double kDriveLowGearPositionKp = 0.85;
    public static final double kDriveLowGearPositionKi = 0.002;
    public static final double kDriveLowGearPositionKd = 100.0;
    public static final double kDriveLowGearPositionKf = .45;
    public static final int kDriveLowGearPositionIZone = 700;
    public static final double kDriveLowGearPositionRampRate = .25; // 48.0 V/s -> 12V in .25s
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
    // TODO: Update for 2018
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
