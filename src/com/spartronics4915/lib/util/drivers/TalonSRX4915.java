package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.Logger;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/* CANTalonw4915 is a portability interface intended to facilitate
 * porting from CTRE CANTalon 2017 libs to CTRE Phoenix 2018.
 * This abstraction is useful as a buffer between all the
 * bad/weird stuff that CTRE does. It allows us to document
 * better, and even implement new features (e.g Talon
 * compatibility with Synthesis).
 *
 * NB: we should endeavor to constrain the interface to those
 *   methods that we feel are useful to subsystems.
 */

// From Phoenix docs:
// What are the units of my sensor?
// Position units are in the natural units of the sensor.
//  Quad (US Digital) 1024 CPR -> 4096
//  CTRE Quad relative  -> 4096
//  CTRE Quad absolute -> 4096
//  Pulse-width encoded position -> 4096 is 100%
//  AndyMark CIMcode    ->  80  (20 pulse -> 80 edges)
//
// Velocity is measured in sensor units per 100ms. This ensures
//  sufficient resolution regardless of the sensing strategy. For
//  example, when using the CTRE Magnetic Encoder, 1u velocity
//  represents 1/4096 of a rotation every 100ms. Generally you
//  can multiply the velocity units by 600/UnitsPerRotation to obtain RPM.
//
// Tachometer velocity measurement is unique in that it measures
//  time directly. As a result, the reported velocity is calculated
//  where 1024 represents a full "rotation". This means that a velocity
//  measurement of 1 represents 1/1024 of a rotation every 100ms
//
// On timeouts:
//  Some routines can optionally wait for a response from the device
//  up to a timeout. If the response is never received, an error code
//  is returned and the Drive Station will receive an error message.
//  Pass 0 to avoid blocking at all. When initializing device on robot-boot,
//  we recommend passing 10ms. When calling routines in the loop of the
//  robot, pass zero to avoid blocking. All configuration routines are
//  prefixed with "config" instead of "set" or "enable". Configuration
//  routines are defined as persistent parameters. Config routines are
//  also factory defaulted by press-and-holding B/C CAL button on boot.
//
// On persistent state:
//  The TalonSRX has many parameters whose values persist across power
//  downs.  One of the biggest time-syncs in Talon development occurs when
//  we only partially specify values for persistent parameters.  When dabbling
//  with new values we may try out some values, then comment them out Because
//  they either had not effect or had a bad effect.   Now, the last value present
//  wins and behavior becomes obscured by the fact that the code is commented
//  out.
//
// On migration:
// https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Migration%20Guide.md
//
//  On factory vs CANTalon4915:
//    We'd like to encapsulate all the CANTalon details here.  Creating named
//    configs is a good thing, but configuring the motors is private business
//    which, if dealt with here means our interface can be more minimal/private.
//
public class TalonSRX4915 implements Sendable, MotorSafety
{

    static final int sInitTimeoutMS = 10;
    static final int sUpdateTimeoutMS = 0; // 0 for no blocking. This is like the old behavior (I think).

    public enum Config
    {
        kCustomMotor(0),
        kDefaultMotor(1),
        kDriveMotor(2),
        kDriveFollowerMotor(3);

        public final int mValue;

        Config(int initValue)
        {
            mValue = initValue;
        }
    };

    /* CANTalon4915 members ------------------------------------------------ */
    static final int sPidIdx = 0; // 0 is primary closed-loop, 1 is cascaded (unused atm)
    static final int sDefaultOrdinal = 0; // This probably does something specific on certain ParamEnums

    ControlMode mControlMode = ControlMode.Disabled;
    ControlMode mLastControlMode = ControlMode.Disabled;
    NeutralMode mNeutralMode = NeutralMode.Brake;
    final int mDeviceId;
    double mLastSetpoint = 0;
    FeedbackDevice mSensor = FeedbackDevice.None; // set via configMotorAndSensor
    int mQuadCodesPerRev = 0; // QuadEncoder codes per revolution, 0 means no encoder
    TalonSRX mTalon = null;
    String mDescription;
    String mSubsystem = "TalonSRX4915"; // for Sendable/LiveWindow
    MotorSafetyHelper mSafetyHelper;

    /* CANTalon4915 methods ------------------------------------------------ */
    public TalonSRX4915(int deviceNumber)
    {
        this(deviceNumber, Config.kDefaultMotor);
    }

    public TalonSRX4915(int deviceNumber, Config c)
    {
        mDeviceId = deviceNumber;
        mDescription = "TalonSRX4915 " + deviceNumber;
        LiveWindow.add(this);
        setName(mDescription);

        CANProbe canProbe = CANProbe.getInstance();
        if (canProbe.validateSRXId(deviceNumber))
        {
            mTalon = new TalonSRX(deviceNumber);
            HAL.report(66, deviceNumber + 1); // from WPI_TalonSRX
            mSafetyHelper = new MotorSafetyHelper(this);
            mSafetyHelper.setExpiration(0.0);
            mSafetyHelper.setSafetyEnabled(false);
            configUniversal(c, sInitTimeoutMS);
        }
    }

    public boolean isValid()
    {
        return mTalon != null;
    }

    public TalonSRX getTalon()
    {
        return mTalon;
    }

    public int getId()
    {
        return mDeviceId;
    }

    // configuration { -----------------------------------------------------------------------
    //
    // configUniversal represents the default/'factory' settings for all our CANTalons.
    // These values should only be changed if you wish them to apply to all robot motors.
    // Alternatively, you should invoke or create special config-groups intended for use
    // by a particular motor usage class (eg driveTrain invokes configOutputPower repeatedly)
    //
    // NB: be *very* careful if you comment out any lines here.
    //   these values are persistent and you can "lock" values
    //   into a motor controller.
    private void configUniversal(Config c, int timeOutMS)
    {
        if (mTalon == null)
            return;

        // power and response ------------------------------------------------------------------
        mTalon.configClosedloopRamp(.5, timeOutMS); // .5 sec to go from 0 to max
        mTalon.configOpenloopRamp(.5, timeOutMS);
        mTalon.configNeutralDeadband(.04, timeOutMS); // output deadband pct 4% (factory default)
        mTalon.configNominalOutputForward(0.0, timeOutMS); // [0, 1]
        mTalon.configNominalOutputReverse(0.0, timeOutMS); // [-1, 0]
        mTalon.configPeakOutputForward(1.0, timeOutMS);
        mTalon.configPeakOutputReverse(-1.0, timeOutMS);

        // current limits are TalonSRX-specific
        // Configure the continuous allowable current-draw (when current limit is enabled).
        // Current limit is activated when current exceeds the peak limit for longer than the
        // peak duration. Then software will limit to the continuous limit. This ensures current
        // limiting while allowing for momentary excess current events.
        // For simpler current-limiting (single threshold) use configContinuousCurrentLimit() and
        // set the peak to zero: configPeakCurrentLimit(0).
        mTalon.enableCurrentLimit(false);
        mTalon.configContinuousCurrentLimit(10, timeOutMS); // 10 amps
        mTalon.configPeakCurrentLimit(50, timeOutMS);
        mTalon.configPeakCurrentDuration(5000, timeOutMS); // milliseconds

        // voltageCompensation:
        //  This is the max voltage to apply to the hbridge when voltage compensation is enabled.
        //  For example, if 10 (volts) is specified and a TalonSRX is commanded to 0.5
        //  (PercentOutput, closed-loop, etc) then the TalonSRX will attempt to apply a
        //  duty-cycle to produce 5V.
        mTalon.enableVoltageCompensation(false);
        mTalon.configVoltageCompSaturation(12.0, timeOutMS);
        mTalon.configVoltageMeasurementFilter(64, timeOutMS);

        // MotionMAGIC sensorUnitsPer100ms used for position-control ----------------------------
        mTalon.configMotionAcceleration(0, timeOutMS);
        mTalon.configMotionCruiseVelocity(0, timeOutMS);
        mTalon.configMotionProfileTrajectoryPeriod(0, timeOutMS);
        mTalon.clearMotionProfileHasUnderrun(timeOutMS);
        mTalon.clearMotionProfileTrajectories();

        // filters and sensors, remote and local --------------------------------------------------
        // no remote feedback filters for now (for remote sensors, like remote, not direct, IMU
        // nb: these are intentionally left blank
        // mTalon.configRemoteFeedbackFilter(0, RemoteSensorSource.Off, 0, 0);
        // mTalon.configRemoteFeedbackFilter(0, RemoteSensorSource.Off, 1, 0);
        // mTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.None, 0, 0);

        // default to no local feedback sensors
        mTalon.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);

        // no forward limit switch
        mTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated,
                LimitSwitchNormal.NormallyOpen, timeOutMS);
        mTalon.configForwardSoftLimitEnable(false, timeOutMS);
        mTalon.configForwardSoftLimitThreshold(0, timeOutMS); // in raw sensor units

        // no reverse limit switch
        mTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated,
                LimitSwitchNormal.NormallyOpen, timeOutMS);
        mTalon.configReverseSoftLimitEnable(false, timeOutMS);
        mTalon.configReverseSoftLimitThreshold(0, timeOutMS);

        // misc non-config, but valuable defaults -----------------------------------------------
        this.setControlMode(ControlMode.PercentOutput);
        mTalon.setInverted(false);
        mTalon.setSensorPhase(false);
        mTalon.setNeutralMode(mNeutralMode);
        mTalon.clearStickyFaults(timeOutMS);

        // feedback rate control ----------------------------------------------------------------
        //  nb: frame periods aren't persistent
        //  setStatusFramePeriod is used to tradeoff CAN bus utilization for feedback
        //  period is measured in milliseconds
        if (c == Config.kDriveMotor)
        {
            // drive wants very tight feedback
            mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, timeOutMS);
            mTalon.configVelocityMeasurementWindow(32, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, timeOutMS);
        }
        else if (c == Config.kDriveFollowerMotor)
        {
            // followers don't need feedback 'tall
            mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, timeOutMS);
            mTalon.configVelocityMeasurementWindow(64, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 1000,
                    timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, timeOutMS);
        }
        else
        {
            // should we be more concerned in the non-drive motor cases?
            mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, timeOutMS);
            mTalon.configVelocityMeasurementWindow(64, timeOutMS);
        }
        // Calling application can opt to speed up the handshaking between the robot API and
        // the controller to increase the download rate of the controller's Motion Profile.
        // Ideally the period should be no more than half the period of a trajectory point
        mTalon.changeMotionControlFramePeriod(100); // millis
    }

    // configMotorAndSensor:
    // sensorPhase:
    //  Sets the phase of the sensor. Use when controller forward/reverse output doesn't
    //  correlate to appropriate forward/reverse reading of sensor. Pick a value so that
    //  positive PercentOutput yields a positive change in sensor. After setting this, user
    //  can freely call SetInvert() with any value.
    public void configMotorAndSensor(boolean invertMotorOutput,
            FeedbackDevice dev, boolean sensorPhase,
            int quadCodesPerRev)
    {
        if (mTalon == null)
            return;
        mSensor = dev;
        mQuadCodesPerRev = quadCodesPerRev;
        mTalon.configSelectedFeedbackSensor(dev, 0/* pidIdx */, sInitTimeoutMS);
        mTalon.setSensorPhase(sensorPhase);
        mTalon.setInverted(invertMotorOutput);
    }
    
    // soft limits depend upon a sensor and are measured in raw sensor units.
    public void configMotorSoftLimits(boolean enableFwd, boolean enableRev,
                                    int sensorUnits)
    {
        mTalon.configForwardSoftLimitEnable(enableFwd, sInitTimeoutMS);
        mTalon.configForwardSoftLimitThreshold(sensorUnits, sInitTimeoutMS);
        mTalon.configReverseSoftLimitEnable(enableRev, sInitTimeoutMS);
        mTalon.configReverseSoftLimitThreshold(sensorUnits, sInitTimeoutMS);  
    }
    
    public void configLimitSwitches(LimitSwitchSource fwdsrc,
                                    LimitSwitchSource revsrc,
            LimitSwitchNormal normallyOpenOrClosed)
    {
        if (mTalon != null)
        {
            mTalon.configForwardLimitSwitchSource(fwdsrc,
                    normallyOpenOrClosed, sInitTimeoutMS);
            mTalon.configReverseLimitSwitchSource(revsrc,
                    normallyOpenOrClosed, sInitTimeoutMS);
        }
    }

    public void configFollower(int masterId, boolean invert)
    {
        if (mTalon == null)
            return;
        mTalon.set(ControlMode.Follower, (double) masterId);
        mTalon.setInverted(invert);
    }

    public void configNominalOutput(double fwd, double rev)
    {
        if (mTalon == null)
            return;
        mTalon.configNominalOutputForward(fwd, sInitTimeoutMS);
        mTalon.configNominalOutputForward(rev, sInitTimeoutMS);
    }

    public void configOutputPower(boolean isOpenLoop,
            double rampRate,
            double nominalFwdOutput, // [0,1]
            double peakFwdOutput, // [0,1]
            double nominalRevOutput, // [-1,0]
            double peakRevOutput) // [-1, 0]
    {
        if (mTalon == null)
            return;
        if (isOpenLoop)
            mTalon.configOpenloopRamp(rampRate, sInitTimeoutMS);
        else
            mTalon.configClosedloopRamp(rampRate, sInitTimeoutMS);
        mTalon.configNominalOutputForward(nominalFwdOutput, sInitTimeoutMS);
        mTalon.configNominalOutputReverse(nominalRevOutput, sInitTimeoutMS);
        mTalon.configPeakOutputForward(peakFwdOutput, sInitTimeoutMS);
        mTalon.configPeakOutputReverse(peakRevOutput, sInitTimeoutMS);
    }
    
    public void configCurrentLimit(boolean enabled, int continuousLimit,
            int peakLimit, int peakDuration )
    {
        if(mTalon != null)
        {
            mTalon.enableCurrentLimit(false);
            mTalon.configContinuousCurrentLimit(continuousLimit, sInitTimeoutMS); // 10 amps
            mTalon.configPeakCurrentLimit(peakLimit, sInitTimeoutMS);
            mTalon.configPeakCurrentDuration(peakDuration, sInitTimeoutMS); // millisecond
        }
    }
    
    public void configMotionMagicRPM(double maxVelocityRPM, double maxAccelRPMPerSec)
    {
        if (mTalon == null)
            return;
        int v = rpmToNativeVelocity(maxVelocityRPM);
        int a = rpmToNativeVelocity(maxAccelRPMPerSec);
        mTalon.configMotionCruiseVelocity(v, sInitTimeoutMS);
        mTalon.configMotionAcceleration(a, sInitTimeoutMS);
        // mTalon.configMotionProfileTrajectoryPeriod(trajPeriod, sUpdateTimeoutMS);
    }

    public void configPID(int slotIdx, double p, double i, double d, double f, int izone,
            double closedLoopRampRate)
    {
        if (mTalon != null)
        {
            mTalon.config_kP(slotIdx, p, sUpdateTimeoutMS);
            mTalon.config_kI(slotIdx, i, sUpdateTimeoutMS);
            mTalon.config_kD(slotIdx, d, sUpdateTimeoutMS);
            mTalon.config_kF(slotIdx, f, sUpdateTimeoutMS);
            mTalon.config_IntegralZone(slotIdx, izone, sUpdateTimeoutMS);
            mTalon.configClosedloopRamp(closedLoopRampRate, sUpdateTimeoutMS); // not a slot-based mod!
            mTalon.setIntegralAccumulator(0.0, 0, sUpdateTimeoutMS);
        }
    }

    public boolean isEnabled()
    {
        if (mTalon != null)
            return mTalon.getControlMode() != ControlMode.Disabled ? true : false;
        else
            return false;
    }

    public boolean isBrakeEnabled()
    {
        return mNeutralMode == NeutralMode.Brake;
    }

    public void setBrakeMode(boolean s)
    {
        if (mTalon != null)
        {
            mNeutralMode = s ? NeutralMode.Brake : NeutralMode.Coast;
            mTalon.setNeutralMode(mNeutralMode);
        }
    }

    public void setInverted(boolean s)
    {
        if (mTalon != null)
        {
            mTalon.setInverted(s);
            ;
        }
    }

    public boolean getInverted()
    {
        if (mTalon != null)
        {
            return mTalon.getInverted();
        }
        else
            return false;
    }

    public double getOutputVoltage()
    {
        if (mTalon != null)
            return mTalon.getMotorOutputVoltage();
        else
            return 0.0;
    }

    public double getOutputCurrent()
    {
        if (mTalon != null)
            return mTalon.getOutputCurrent();
        else
            return 0.0;
    }

    public String dumpState()
    {
        if (mTalon != null)
        {
            StringBuilder sb = new StringBuilder()
                    .append("TalonSRX4915 state for ")
                    .append(mDescription)
                    .append("------------- {\n")
                    .append("  firmware version:")
                    .append(Integer.toHexString(mTalon.getFirmwareVersion()))
                    .append("\n")
                    .append(dumpPowerState(false/* means !terse */))
                    .append("\n")
                    .append("  feedback sensor type:")
                    .append(mTalon.configGetParameter(ParamEnum.eFeedbackSensorType, 0,
                            sInitTimeoutMS))
                    .append("  quadIdxPolarity:")
                    .append(mTalon.configGetParameter(ParamEnum.eQuadIdxPolarity, 0,
                            sInitTimeoutMS))
                    .append("  limit switch source:")
                    .append(mTalon.configGetParameter(ParamEnum.eLimitSwitchSource, 0,
                            sInitTimeoutMS))
                    .append("\n")
                    .append("  fwd soft limit enable:")
                    .append(mTalon.configGetParameter(ParamEnum.eForwardSoftLimitEnable, 0,
                            sInitTimeoutMS))
                    .append("\n")
                    .append("  fwd soft limit threshold:")
                    .append(mTalon.configGetParameter(ParamEnum.eForwardSoftLimitThreshold, 0,
                            sInitTimeoutMS))
                    .append("\n")
                    .append("  rev soft limit enable:")
                    .append(mTalon.configGetParameter(ParamEnum.eReverseSoftLimitEnable, 0,
                            sInitTimeoutMS))
                    .append("\n")
                    .append("  rev soft limit threshold:")
                    .append(mTalon.configGetParameter(ParamEnum.eReverseSoftLimitThreshold, 0,
                            sInitTimeoutMS))
                    .append("\n")
                    .append("  faults: ")
                    .append(dumpFaults())
                    .append("\n")
                    .append("--------------}\n");
            ;
            return sb.toString();
        }
        else
        {
            return "Talon " + mDeviceId + " was not found on the CAN bus";
        }
    }

    public String dumpPowerState(boolean terse)
    {
        StringBuilder sb = new StringBuilder()
                .append("  inverted:")
                .append(mTalon.getInverted())
                .append(",  following:")
                .append(mTalon.getControlMode() != ControlMode.Follower)
                .append("\n")
                .append("  openLoopRampRate:")
                .append(mTalon.configGetParameter(ParamEnum.eOpenloopRamp, 0, sInitTimeoutMS))
                .append("  output peak +,-:")
                .append(mTalon.configGetParameter(ParamEnum.ePeakPosOutput, 0, sInitTimeoutMS))
                .append(", ")
                .append(mTalon.configGetParameter(ParamEnum.ePeakNegOutput, 0, sInitTimeoutMS))
                .append(",  nominal +,-:")
                .append(mTalon.configGetParameter(ParamEnum.eNominalPosOutput, 0, sInitTimeoutMS))
                .append(", ")
                .append(mTalon.configGetParameter(ParamEnum.eNominalNegOutput, 0, sInitTimeoutMS));

        if (!terse)
        {
            sb.append("\n");
            sb.append("  closedLoopRampRate:");
            sb.append(mTalon.configGetParameter(ParamEnum.eClosedloopRamp, 0, sInitTimeoutMS));
            sb.append("\n");
            sb.append("  neutral deadband:");
            sb.append(mTalon.configGetParameter(ParamEnum.eNeutralDeadband, 0,
                    sInitTimeoutMS));
            sb.append("\n");
            sb.append("  current limits (if enabled) peak:");
            sb.append(mTalon.configGetParameter(ParamEnum.ePeakCurrentLimitAmps, 0,
                    sInitTimeoutMS));
            sb.append(",  continuous:");
            sb.append(mTalon.configGetParameter(ParamEnum.eContinuousCurrentLimitAmps, 0,
                    sInitTimeoutMS));
            sb.append("\n");
            sb.append("  output voltage:");
            sb.append(getOutputVoltage());
            sb.append("  current:");
            sb.append(getOutputCurrent());
        }
        return sb.toString();
    }

    public String dumpPIDState(int slotIdx)
    {
        StringBuilder sb = new StringBuilder()
                .append(" Motion Magic Cruise Velocity:")
                .append(mTalon.configGetParameter(ParamEnum.eMotMag_VelCruise, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("  Motion Magic Max Accel:")
                .append(mTalon.configGetParameter(ParamEnum.eMotMag_Accel, slotIdx, sInitTimeoutMS))
                .append("\n")
                .append("  slot0 P:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_P, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       I:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_I, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       D:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_D, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       F:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_F, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       allowable error:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_AllowableErr, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       izone:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_IZone, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       maxiaccum:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_MaxIAccum, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("  slot1 P:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_P, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       I:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_I, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       D:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_D, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       F:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_F, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       allowable error:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_AllowableErr, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       izone:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_IZone, slotIdx,
                        sInitTimeoutMS))
                .append("\n")
                .append("       maxiaccum:")
                .append(mTalon.configGetParameter(ParamEnum.eProfileParamSlot_MaxIAccum, slotIdx,
                        sInitTimeoutMS));
        return sb.toString();
    }

    public String dumpFaults()
    {
        StringBuilder sb = new StringBuilder();
        if (getFaultHardwareFailure())
            sb.append("hardware! ");
        if (getFaultUnderVoltage())
            sb.append("underVoltage ");
        if (getStickyFaultUnderVoltage())
            sb.append("underVoltageSticky ");
        if (getFaultFwdLim())
            sb.append("fwdLimFault ");
        if (getStickyFaultFwdLim())
            sb.append("fwdLimFaultSticky ");
        if (getFaultFwdSoftLim())
            sb.append("fwdSoftLimit ");
        if (getStickyFaultFwdSoftLim())
            sb.append("fwdSoftLimFaultSticky ");
        if (getFaultRevLim())
            sb.append("revLimFault ");
        if (getStickyFaultRevLim())
            sb.append("revLimFaultSticky ");
        if (getFaultRevSoftLim())
            sb.append("revSoftLimit ");
        if (getStickyFaultRevSoftLim())
            sb.append("revSoftLimFaultSticky ");
        return sb.toString();
    }

    // } configuration

    /*
     * TalonSRX dispatch
     * -------------------------------------------------------------------
     */
    public void setControlMode(ControlMode m)
    {
        this.mControlMode = m; // in SRX mode, set() requires controlmode
    }

    public void setVelocityRPM(double rpm)
    {
        if (mTalon == null)
            return;
        this.set(ControlMode.Velocity, rpmToNativeVelocity(rpm));
    }

    public void setPositionRotations(double rots)
    {
        if (mTalon == null)
            return;
        // control mode might be MotionMagic or Position
        this.set(mControlMode, rotationsToNative(rots));
    }

    /**
     * Sets the output on the Talon, with the mode specified explicitly.
     * This mimics/intercepts the new-style method, so that we can maintain
     * state in the wrapper subsystem and implement motor safety.
     */
    public void set(ControlMode m, double value)
    {
        if (mTalon == null)
            return;
        mControlMode = m;
        if (mSafetyHelper != null)
            mSafetyHelper.feed();
        if (value != mLastSetpoint || mControlMode != mLastControlMode)
        {
            mTalon.set(mControlMode, value);
            mLastSetpoint = value;
            mLastControlMode = mControlMode;
        }
    }

    /**
     * low level set is here for compatibility with old API.
     * units depend on the control mode which is why the higher level
     * variants are preferred. Ultimately high level entrypoints
     * are expected to route here, since motor safety occurs here.
     **/
    public void set(double value)
    {
        if (mTalon == null)
            return;
        if (mSafetyHelper != null)
            mSafetyHelper.feed();
        if (value != mLastSetpoint || mControlMode != mLastControlMode)
        {
            mTalon.set(mControlMode, value);
            mLastSetpoint = value;
            mLastControlMode = mControlMode;
        }
    }

    public double get()
    {
        return mLastSetpoint;
    }

    public double getSetpointRPM()
    {
        if (mControlMode == ControlMode.Velocity)
            return nativeVelocityToRPM(mLastSetpoint);
        else
            return 0;
    }

    public double getSetpointRotations()
    {
        if (mControlMode == ControlMode.Position || mControlMode == ControlMode.MotionMagic)
            return nativeToRotations(mLastSetpoint);
        else
            return 0;
    }

    public double getSensorVelocityRPM()
    {
        if (mTalon != null)
            return nativeVelocityToRPM(mTalon.getSelectedSensorVelocity(sPidIdx));
        else
            return 0;
    }

    public int getSensorVelocityNative()
    {
        if (mTalon != null)
            return mTalon.getSelectedSensorVelocity(sPidIdx);
        else
            return 0;
    }

    public double getSensorPositionRotations()
    {
        if (mTalon != null)
        {
            return nativeToRotations(mTalon.getSelectedSensorPosition(sPidIdx));
        }
        else
            return 0.0;
    }

    public double getSensorPositionNative()
    {
        if (mTalon != null)
        {
            return mTalon.getSelectedSensorPosition(sPidIdx);
        }
        else
            return 0.0;
    }

    public double getActiveTrajectoryPosition() // for MotionMagic
    {
        if (mTalon != null)
            return mTalon.getActiveTrajectoryPosition();
        else
            return 0.0;
    }

    // sensor/encoder set/query ----------------------------------------
    public void resetSensor()
    {
        setSensorPositionNative(0);
    }

    private void setSensorPositionNative(int pos)
    {
        if (mTalon != null)
        {
            mTalon.getSensorCollection().setQuadraturePosition(pos, sUpdateTimeoutMS);
        }
    }

    // limit switch ------------------------------------------------------
    public boolean isFwdLimitSwitchClosed()
    {
        if (mTalon != null)
        {
            return mTalon.getSensorCollection().isFwdLimitSwitchClosed();
        }
        else
            return true;
    }

    public boolean isFwdSoftLimitEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eForwardSoftLimitEnable, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
        else
            return true;
    }

    public boolean isRevLimitSwitchClosed()
    {
        if (mTalon != null)
        {
            return mTalon.getSensorCollection().isRevLimitSwitchClosed();
        }
        else
            return true;
    }

    public boolean isRevSoftLimitEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eReverseSoftLimitEnable, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
        else
            return true;
    }

    // pid support --------------------------------------------------------
    // Selects which profile slot to use for closed-loop control.
    //  slotIdx is an 'arbitrary' stash identifier
    //  pidIdx is 0 for primary closed-loop and 1 for cascaded closed-loop
    //      so far we're not doing any cascaded closed-loop
    public void selectProfileSlot(int slotIdx)
    {
        if (mTalon != null)
        {
            mTalon.selectProfileSlot(slotIdx, sPidIdx);
        }
    }

    public void resetClosedLoopState()
    {
        if (mTalon != null)
        {
            mTalon.setIntegralAccumulator(0, sPidIdx, sUpdateTimeoutMS);
            mTalon.clearMotionProfileHasUnderrun(sUpdateTimeoutMS);
            mTalon.clearStickyFaults(sUpdateTimeoutMS);
        }
    }

    public double getClosedLoopError()
    {
        if (mTalon != null)
            return mTalon.getClosedLoopError(sPidIdx);
        else
            return 0.0;
    }

    public boolean isClosedLoopEnabled()
    {
        boolean result = false;
        if (mTalon != null)
        {
            ControlMode m = mTalon.getControlMode();
            if (m != ControlMode.Disabled &&
                    m != ControlMode.PercentOutput &&
                    m != ControlMode.Follower)
            {
                result = true;
            }
        }
        return result;
    }

    /*
     * unit conversions
     * -------------------------------------------------------------
     */
    /**
     * Convert RPM (rotations per min) to Native Unit velocity.
     * Native velocity is Native Units per 100 milliseconds.
     * configMotorAndEncoder must have been called for
     * this to work!
     */
    public int rpmToNativeVelocity(double rpm)
    {
        if (mQuadCodesPerRev == 0)
            return (int) Math.round(rpm);
        double rotationsPer100MS = (rpm / 60) / 10;
        return (int) Math.round(rotationsPer100MS * mQuadCodesPerRev);
    }

    /**
     * Convert Native Unit velocity to RPM (rotations per minute).
     * Native velocity is Native Units per 100 milliseconds.
     * configMotorAndEncoder must have been called for
     * this to work!
     *
     * The speed units will be in the sensor's native ticks per 100ms.
     * For analog sensors, 3.3V corresponds to 1023 units. So a speed
     * of 200 equates to ~0.645 dV per 100ms or 6.451 dV per second.
     * If this is an analog encoder, that likely means 1.9548 rotations
     * per sec. For quadrature encoders, each unit corresponds a quadrature
     * edge (4X). So a 250 count encoder will produce 1000 edge events
     * per rotation. An example speed of 200 would then equate to 20%
     * of a rotation per 100ms, or 10 rotations per second.
     */
    private int nativeVelocityToRPM(double nativeVelocity)
    {
        if (mQuadCodesPerRev == 0)
            return (int) Math.round(nativeVelocity);
        double nativeUnitsPerMinute = nativeVelocity * 10 * 60;
        return (int) Math.round(nativeUnitsPerMinute / mQuadCodesPerRev);
    }

    /**
     * Convert encoder codes (encoder specific)to wheel rotations.
     * configMotorAndEncoder() must have been called for this to work!
     *
     * @param Absolute encoder codes (use {@link nativeVelocityToRpm} for
     *        non-absolute units)
     * @return Absolute wheel rotations
     */
    private double nativeToRotations(double codes)
    {
        if (mQuadCodesPerRev == 0)
            return codes;
        return codes / mQuadCodesPerRev;
    }

    private double rotationsToNative(double rots)
    {
        if (mQuadCodesPerRev == 0)
            return rots;
        else
            return mQuadCodesPerRev * rots;
    }

    // misc fault detection --------------------------------------------------------------
    /**
     * Check if there's a hardware failure.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a hardware failure.
     */
    private boolean getFaultHardwareFailure()
    {
        if (mTalon != null)
        {
            Faults faults = new Faults();
            mTalon.getFaults(faults);
            return faults.HardwareFailure;
        }
        else
            return false;
    }

    /**
     * Check if the voltage in went under 6.5V.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a hardware failure.
     */
    private boolean getFaultUnderVoltage()
    {
        if (mTalon != null)
        {
            Faults faults = new Faults();
            mTalon.getFaults(faults);
            return faults.UnderVoltage;
        }
        else
            return false;
    }

    /**
     * Check if voltage dropped below 6.5V. A sticky fault is
     * just one that persists.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is under voltage or not.
     */
    private boolean getStickyFaultUnderVoltage()
    {
        if (mTalon != null)
        {
            StickyFaults faults = new StickyFaults();
            mTalon.getStickyFaults(faults);
            return faults.UnderVoltage;
        }
        else
            return false;
    }

    /**
     * Check if there's a forward limit switch issue.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a hardware failure.
     */
    private boolean getFaultFwdLim()
    {
        if (mTalon != null)
        {
            Faults faults = new Faults();
            mTalon.getFaults(faults);
            return faults.ForwardLimitSwitch;
        }
        else
            return false;
    }

    /**
     * Check if there's a forward limit switch issue.
     * A sticky fault is just one that persists.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a limit switch issue.
     */
    private boolean getStickyFaultFwdLim()
    {
        if (mTalon != null)
        {
            StickyFaults faults = new StickyFaults();
            mTalon.getStickyFaults(faults);
            return faults.ForwardLimitSwitch;
        }
        else
            return false;
    }

    /**
     * Check if there's a forward soft limit issue. I think forward soft limit
     * refers to a limit switch, but CTRE docs are not exactly good
     * or comprehensive.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a hardware failure.
     */
    private boolean getFaultFwdSoftLim()
    {
        if (mTalon != null)
        {
            Faults faults = new Faults();
            mTalon.getFaults(faults);
            return faults.ForwardSoftLimit;
        }
        else
            return false;
    }

    /**
     * Check if there's a forward soft limit issue. I think forward soft limit
     * refers to a limit switch, but CTRE docs are not exactly good
     * or comprehensive.
     * A sticky fault is just one that persists.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a limit switch issue.
     */
    private boolean getStickyFaultFwdSoftLim()
    {
        if (mTalon != null)
        {
            StickyFaults faults = new StickyFaults();
            mTalon.getStickyFaults(faults);
            return faults.ForwardSoftLimit;
        }
        else
            return false;
    }

    /**
     * Check if there's a reverse limit switch issue.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there limit switch issue.
     */
    private boolean getFaultRevSoftLim()
    {
        if (mTalon != null)
        {
            Faults faults = new Faults();
            mTalon.getFaults(faults);
            return faults.ReverseSoftLimit;
        }
        else
            return false;
    }

    /**
     * Check if there's a reverse limit switch issue.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there limit switch issue.
     */
    private boolean getFaultRevLim()
    {
        if (mTalon != null)
        {
            Faults faults = new Faults();
            mTalon.getFaults(faults);
            return faults.ReverseLimitSwitch;
        }
        else
            return false;
    }

    /**
     * Check if there's a reverse limit switch issue.
     * A sticky fault is just one that persists.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a limit switch issue.
     */
    private boolean getStickyFaultRevLim()
    {
        if (mTalon != null)
        {
            StickyFaults faults = new StickyFaults();
            mTalon.getStickyFaults(faults);
            return faults.ReverseLimitSwitch;
        }
        else
            return false;
    }

    /**
     * Check if there's a reverse limit switch issue.
     * A sticky fault is just one that persists.
     *
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     *
     * @return Is there a limit switch issue.
     */
    private boolean getStickyFaultRevSoftLim()
    {
        if (mTalon != null)
        {
            StickyFaults faults = new StickyFaults();
            mTalon.getStickyFaults(faults);
            return faults.ReverseSoftLimit;
        }
        else
            return false;
    }

    // MotorSafety Interface { -------------------------------------------------------------
    @Override
    public String getDescription()
    {
        return mDescription;
    }

    @Override
    public double getExpiration()
    {
        if (mSafetyHelper != null)
            return mSafetyHelper.getExpiration();
        else
            return 0.0;
    }

    @Override
    public boolean isAlive()
    {
        if (mSafetyHelper != null)
            return mSafetyHelper.isAlive();
        else
            return false;
    }

    @Override
    public boolean isSafetyEnabled()
    {
        if (mSafetyHelper != null)
            return mSafetyHelper.isSafetyEnabled();
        else
            return false;
    }

    @Override
    public void setExpiration(double arg0)
    {
        if (mSafetyHelper != null)
            mSafetyHelper.setExpiration(arg0);
    }

    @Override
    public void setSafetyEnabled(boolean arg0)
    {
        if (mSafetyHelper != null)
            mSafetyHelper.setSafetyEnabled(arg0);

    }

    @Override
    public void stopMotor()
    {
        if (mTalon != null)
        {
            mTalon.neutralOutput();
        }
    }
    // } MotorSafety

    // Sendable interface {
    @Override
    public String getName()
    {
        return mDescription;
    }

    @Override
    public String getSubsystem()
    {
        return mSubsystem;
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("Speed Controller");
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    @Override
    public void setName(String arg0)
    {
        mDescription = arg0;
    }

    @Override
    public void setSubsystem(String arg0)
    {
        mSubsystem = arg0;
    }

    // } Sendable Interface

}
