package com.spartronics4915.frc2018;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.spartronics4915.lib.util.ConstantsBase;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;
import com.spartronics4915.lib.util.math.PolynomialRegression;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants
 * determined through calibrations.
 */
public class Constants extends ConstantsBase
{

    public static double kLooperDt = 0.005;

    // Target parameters
    // Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
    // Section 3.13
    // ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
    // Parts GE-17203-FLAT and GE-17371 (sheet 7)
    public static double kBoilerTargetTopHeight = 88.0;
    public static double kBoilerRadius = 8;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static double kDriveWheelDiameterInches = 6;
    public static double kTrackWidthInches = 27.75;
    public static double kTrackScrubFactor = 0.624;

    // Geometry
    public static double kCenterToFrontBumperDistance = 14.1875;
    public static double kCenterToIntakeDistance = 20.9675;
    public static double kCenterToRearBumperDistance = 14.1875;
    public static double kCenterToSideBumperDistance = 15.75;

    /* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 6.0;
    public static double kDriveHighGearVelocityKf = .15;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive position loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 0.85;
    public static double kDriveLowGearPositionKi = 0.002;
    public static double kDriveLowGearPositionKd = 100.0;
    public static double kDriveLowGearPositionKf = .45;
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 48.0; // V/s
    public static double kDriveLowGearNominalOutput = 0.5; // V
    public static double kDriveLowGearMaxVelocity = 3.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
                                                                                                               // in RPM
    public static double kDriveLowGearMaxAccel = 15.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;

    /* TALONS */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    // Drive
    public static final int kLeftDriveMasterId = 3;
    public static final int kLeftDriveSlaveId = 1;
    public static final int kRightDriveMasterId = 4;
    public static final int kRightDriverSlaveId = 2;
    public static final int kIMUTalonId = 6;
    public static final int kEncoderCodesPerRev = 250; // Not ticks, codes; ticks would be kEncoderCodesPerRev * 4 (for quadrature encoders at least)
    public static final int kNumCANDevices = 8;

    // Solenoids
    public static final int kShifterSolenoidId = 0; // PCM 0, Solenoid 0
    public static final int kIntakeDeploySolenoidId = 1; // PCM 0, Solenoid 1
    public static final int kHopperSolenoidId = 2; // PCM 0, Solenoid 2
    public static final int kGearWristSolenoid = 7; // PCM 0, Solenoid 7

    // Analog Inputs
    public static int kLEDOnId = 2;

    // Digital Outputs
    public static int kGreenLEDId = 9;
    public static int kRangeLEDId = 8;

    // Phone
    public static int kAndroidAppTcpPort = 8254;

    // Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 0.05; // inches
    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 20.00;
    public static double kPathFollowingProfileKi = 0.08;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 1.0;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;

    // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset = -3.3211;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 20.9;
    public static double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
    public static double kCameraYawAngleDegrees = 0.0;
    public static double kCameraDeadband = 0.0;

    /* AIM PARAMETERS */

    public static double kDefaultShootingDistanceInches = 95.8;
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = true; // Change to 'true' to use the best-fit polynomial
                                                                // instead.
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double kShooterOptimalRange = 100.0;
    public static double kShooterOptimalRangeFloor = 95.0;
    public static double kShooterOptimalRangeCeiling = 105.0;

    public static double kShooterAbsoluteRangeFloor = 90.0;
    public static double kShooterAbsoluteRangeCeiling = 130.0;

    public static double[][] kFlywheelDistanceRpmValues = {
            // At champs 4/27
            {90.0, 2890.0},
            {95.0, 2940.0},
            {100.0, 2990.0},
            {105.0, 3025.0},
            {110.0, 3075.0},
            {115.0, 3125.0},
            {120.0, 3175.0},
            {125.0, 3225.0},
            {130.0, 3275.0},
    };

    static
    {
        for (double[] pair : kFlywheelDistanceRpmValues)
        {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        kDefaultShootingRPM = kFlywheelAutoAimMap
                .getInterpolated(new InterpolatingDouble(Constants.kDefaultShootingDistanceInches)).value;

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelDistanceRpmValues, 2);
    }

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
