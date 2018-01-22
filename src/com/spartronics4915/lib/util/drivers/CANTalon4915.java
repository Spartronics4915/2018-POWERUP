package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.lib.util.CANProbe;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
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
public class CANTalon4915 implements Sendable, MotorSafety
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

    /* CANTalon4915  members ------------------------------------------------*/
    static final int sPidIdx = 0; // 0 is primary closed-loop, 1 is cascaded (unused atm)
    static final int sDefaultOrdinal = 0; // This probably does something specific on certain ParamEnums

    ControlMode mControlMode = ControlMode.Disabled;
    ControlMode mLastControlMode = null;
    NeutralMode mNeutralMode;
    int mDeviceId;
    double mLastSetpoint = 0;
    int mMaxVolts = 12;
    int mCodesPerRevolution; // Encoder codes per revolution
    TalonSRX mTalon = null;
    String mDescription;
    String mSubsystem = "Ungrouped"; // for Sendable
    MotorSafetyHelper mSafetyHelper;

    /* CANTalon4915  methods ------------------------------------------------*/
    public CANTalon4915(int deviceNumber)
    {
        this(deviceNumber,  Config.kDefaultMotor);
    }
    
    public CANTalon4915(int deviceNumber, Config c)
    {
        mDeviceId = deviceNumber;
        mDescription = "TalonSRX4915 " + deviceNumber;
        LiveWindow.add(this);
        setName(mDescription);

        CANProbe canProbe = CANProbe.getInstance();
        if (canProbe.validateSRXId(deviceNumber))
        {
            mTalon = new TalonSRX(deviceNumber);
            HAL.report(66,  deviceNumber+1); // from WPI_TalonSRX
            mSafetyHelper = new MotorSafetyHelper(this);
            mSafetyHelper.setExpiration(0.0);
            mSafetyHelper.setSafetyEnabled(false);
            configGeneral(c, sInitTimeoutMS);
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
    
    // configuration { -----------------------------------------------------------------------
    private void configGeneral(Config c, int timeOutMS)
    {
        if(mTalon == null) return;
        
        // power and response ------------------------------------------------------------------
        mTalon.configClosedloopRamp(.5, timeOutMS); // .5 sec to go from 0 to max
        mTalon.configOpenloopRamp(.5,  timeOutMS);
        mTalon.configNeutralDeadband(.04,  timeOutMS); // output deadband pct 4% (factory default)
        mTalon.configNominalOutputForward(1.0, timeOutMS);  // [0, 1]
        mTalon.configNominalOutputReverse(-1.0, timeOutMS); // [-1, 0]
        mTalon.configPeakOutputForward(0.5, timeOutMS);
        mTalon.configPeakOutputReverse(-0.5, timeOutMS);
        
        // current limits are TalonSRX-specific
        // Configure the continuous allowable current-draw (when current limit is enabled).
        // Current limit is activated when current exceeds the peak limit for longer than the 
        // peak duration. Then software will limit to the continuous limit. This ensures current 
        // limiting while allowing for momentary excess current events.
        // For simpler current-limiting (single threshold) use configContinuousCurrentLimit() and
        // set the peak to zero: configPeakCurrentLimit(0).
        mTalon.configContinuousCurrentLimit(30, timeOutMS); // 30 amps
        mTalon.configPeakCurrentLimit(0, timeOutMS);
        mTalon.configPeakCurrentDuration(5000, timeOutMS); // milliseconds
        mTalon.enableCurrentLimit(false);
               
        // voltageCompSaturation:
        //  This is the max voltage to apply to the hbridge when voltage compensation is enabled. 
        //  For example, if 10 (volts) is specified and a TalonSRX is commanded to 0.5 
        //  (PercentOutput, closed-loop, etc) then the TalonSRX will attempt to apply a 
        //  duty-cycle to produce 5V.
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
        mTalon.configRemoteFeedbackFilter(0, RemoteSensorSource.Off, 0, 0);
        mTalon.configRemoteFeedbackFilter(0, RemoteSensorSource.Off, 1, 0);
        mTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.None, 0, 0);
        
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
        mTalon.setNeutralMode(NeutralMode.Brake);
        mTalon.clearStickyFaults(timeOutMS);
       
        // feedback rate control ----------------------------------------------------------------
        //  nb: frame periods aren't persistent
        //  setStatusFramePeriod is used to tradeoff CAN bus utilization for feedback
        //  period is measured in milliseconds
        if(c == Config.kDriveMotor)
        {
            // drive wants very tight feedback
            mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, timeOutMS);
            mTalon.configVelocityMeasurementWindow(32, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, timeOutMS);
        }
        else
        if(c == Config.kDriveFollowerMotor)
        {
            // followers don't need feedback 'tall
            mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, timeOutMS);
            mTalon.configVelocityMeasurementWindow(64, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 1000, timeOutMS);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 1000, timeOutMS);
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
    
    // configEncoder:
    //
    // pidIdx:
    // 
    // sensorPhase:
    //  Sets the phase of the sensor. Use when controller forward/reverse output doesn't 
    //  correlate to appropriate forward/reverse reading of sensor. Pick a value so that 
    //  positive PercentOutput yields a positive change in sensor. After setting this, user 
    //  can freely call SetInvert() with any value.
    public void configEncoder(int pidIdx, FeedbackDevice dev, boolean sensorPhase, 
            boolean isInverted, int encoderCodesPerRev)
    {
        if(mTalon == null) return;
        mTalon.configSelectedFeedbackSensor(dev, pidIdx, sInitTimeoutMS);
        mTalon.setSensorPhase(sensorPhase);
        this.setEncoderCodesPerRev(encoderCodesPerRev);
        mTalon.setInverted(isInverted);
    }
    
    public void configNominalOutput(double fwd, double rev)
    {
        if(mTalon == null) return;
        mTalon.configNominalOutputForward(fwd, sInitTimeoutMS);
        mTalon.configNominalOutputForward(rev, sInitTimeoutMS);
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
            mTalon.configClosedloopRamp(closedLoopRampRate, sUpdateTimeoutMS); // XXX: not a slot-based mod!
            // mTalon.configOpenloopRamp(newRampRate, sUpdateTimeoutMS);
        }
    }
    
    public void configMotionMagic(int maxVelocity, int maxAcceleration)
    {
        if(mTalon == null) return;
        mTalon.configMotionCruiseVelocity(maxVelocity, sUpdateTimeoutMS);
        mTalon.configMotionAcceleration(maxAcceleration, sUpdateTimeoutMS);
        // mTalon.configMotionProfileTrajectoryPeriod(trajPeriod, sUpdateTimeoutMS);
    }

    public String dumpState()
    {
        if (mTalon != null)
        {
            StringBuilder sb = new StringBuilder()
                         .append("firmware version:")
                             .append(Integer.toHexString(mTalon.getFirmwareVersion()))
                             .append("\n")
                         ;
            return sb.toString();
         }
        else
        {
            return "Talon " + mDeviceId + " was not found on the CAN bus";
        }
    }
    
    // } configuration

    // MotorSafety Interface { -------------------------------------------------------------
    @Override
    public String getDescription()
    {
        return mDescription;
    }

    @Override
    public double getExpiration()
    {
        if(mSafetyHelper != null)
            return mSafetyHelper.getExpiration();
        else
            return 0.0;
    }

    @Override
    public boolean isAlive()
    {
        if(mSafetyHelper != null)
            return mSafetyHelper.isAlive();
        else
            return false;
    }

    @Override
    public boolean isSafetyEnabled()
    {
        if(mSafetyHelper != null)
            return mSafetyHelper.isSafetyEnabled();
        else
            return false;
    }

    @Override
    public void setExpiration(double arg0)
    {
        if(mSafetyHelper != null)
            mSafetyHelper.setExpiration(arg0);
    }

    @Override
    public void setSafetyEnabled(boolean arg0)
    {
        if(mSafetyHelper != null)
            mSafetyHelper.setSafetyEnabled(arg0);

    }

    @Override
    public void stopMotor()
    {
        if(mTalon != null)
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


    /* TalonSRX dispatch -------------------------------------------------------------------*/
    public double getOutputCurrent()
    {
        if(mTalon != null)
            return mTalon.getOutputCurrent();
        else
            return 0.0;
    }

    /**
     * Converts RPM to Native Unit velocity.
     * RPM is rotations per minute.
     * Native velocity is Native Units per 100 milliseconds.
     * <b>configEncoderCodesPerRev must have been called for
     * this to work!</b>
     *
     * @param Rotations per minute
     * @return Native Units per 100 milliseconds
     */
    private int rpmToNativeVelocity(double rpm)
    {
        if (mCodesPerRevolution == 0)
            return (int) Math.round(rpm);
        double rotationsPer100MS = (rpm / 60) / 10;
        return (int) Math.round(rotationsPer100MS * mCodesPerRevolution);
    }

    /**
     * Converts Native Unit velocity to RPM.
     * RPM is rotations per minute.
     * Native velocity is Native Units per 100 milliseconds.
     * <b>configEncoderCodesPerRev must have been called for
     * this to work!</b>
     *
     * @param Native unit velocity
     * @return Rotations per minute
     */
    private int nativeVelocityToRpm(double nativeVelocity)
    {
        if (mCodesPerRevolution == 0)
            return (int) Math.round(nativeVelocity);
        double nativeUnitsPerMinute = nativeVelocity * 10 * 60;
        return (int) Math.round(nativeUnitsPerMinute / mCodesPerRevolution);
    }

    /**
     * Converts encoder codes (encoder specific)
     * to wheel rotations.
     * <b>configEncoderCodesPerRev must have been called for
     * this to work!</b>
     *
     * @param Absolute encoder codes (use {@link nativeVelocityToRpm} for
     *        non-absolute units)
     * @return Absolute wheel rotations
     */
    private double encoderCodesToRotations(double codes)
    {
        if (mCodesPerRevolution == 0)
            return codes;
        return codes / mCodesPerRevolution;
    }

    /**
     * Sets the output on the Talon, depending on the mode you're in.
     * This is basically the only thing the new API does better,
     * IMO. You need to maintain state with the old API, and this
     * preserves that behavior.
     *
     * @param The output depending on the ControlMode you've set the motor to.
     */
    public void set(double value)
    {
        if (mTalon != null)
        {
            if(mSafetyHelper != null)
                mSafetyHelper.feed();
            if (value != mLastSetpoint || mControlMode != mLastControlMode)
            {
                mTalon.set(mControlMode, value);
                mLastSetpoint = value;
                mLastControlMode = mControlMode;
            }
        }
    }

     /**
     * Sets the output on the Talon, with the mode specified explicitly.
     * This overrides the new-style method, so that we can maintain
     * state in the wrapper subsystem.
     *
     * @param A ControlMode dependent setpoint.
     * @see set(double)
     */
    public void set(ControlMode m, double value)
    {
        if (mTalon != null)
        {
            mControlMode = m;
            this.set(value); // route to this.set above
        }
    }

    public double get()
    {
        return mLastSetpoint;
    }

    public void setControlMode(ControlMode m)
    {
        this.mControlMode = m; // in SRX mode, set() requires controlmode
    }

     private void setEncoderCodesPerRev(int cpr)
    {
        mCodesPerRevolution = cpr;
    }

    public void setEncPosition(int p)
    {
        if (mTalon != null)
        {
            mTalon.getSensorCollection().setQuadraturePosition(p, sUpdateTimeoutMS);
        }
    }

    private void setMotionMagicAcceleration(double motMagicAccel)
    {
        if (mTalon != null)
        {
            mTalon.configMotionAcceleration(rpmToNativeVelocity(motMagicAccel), sUpdateTimeoutMS);
        }
    }

    private void setMotionMagicCruiseVelocity(double kDriveLowGearMaxVelocity)
    {
        if (mTalon != null)
        {
            mTalon.configMotionCruiseVelocity(rpmToNativeVelocity(kDriveLowGearMaxVelocity),
                    sUpdateTimeoutMS);
        }
    }

    private void clearIAccum()
    {
        if (mTalon != null)
            mTalon.setIntegralAccumulator(0, sPidIdx, sUpdateTimeoutMS);
    }

    private void clearMotionProfileHasUnderrun()
    {
        if (mTalon != null)
            mTalon.clearMotionProfileHasUnderrun(sUpdateTimeoutMS);
    }

    private void clearStickyFaults()
    {
        if (mTalon != null)
            mTalon.clearStickyFaults(sUpdateTimeoutMS);
    }

    public void enableBrakeMode(boolean s)
    {
        if (mTalon != null)
        {
            mTalon.setNeutralMode(s ? NeutralMode.Brake : NeutralMode.Coast);
        }
    }

    /**
     * Reverses motor output.
     *
     * @param Is inverted or not.
     */
    public void reverseOutput(boolean s)
    {
        if (mTalon != null)
        {
            mTalon.setInverted(s);
        }
    }

    public void setInverted(boolean isInverted)
    {
        if (mTalon != null)
        {
           mTalon.setInverted(isInverted);
        }
    }

    public void setPosition(int d)
    {
        if (mTalon != null)
        {
            mTalon.getSensorCollection().setAnalogPosition(d, sUpdateTimeoutMS);
        }
    }
    
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

    public void setPulseWidthPosition(int p)
    {
        if (mTalon != null)
        {
            mTalon.getSensorCollection().setPulseWidthPosition(p, sUpdateTimeoutMS);
        }
    }

    public void setStatusFrameRateMs(StatusFrameEnhanced statFrame, int rate)
    {
        if (mTalon != null)
        {
            mTalon.setStatusFramePeriod(statFrame, rate, sUpdateTimeoutMS);
        }
    }

    public void reverseSensor(boolean s)
    {
        if (mTalon != null)
        {
            mTalon.setSensorPhase(s);
        }
    }

    public void setAnalogPosition(int pos)
    {
        if (mTalon != null)
        {
            mTalon.getSensorCollection().setAnalogPosition(pos, sUpdateTimeoutMS);
        }
    }

    /**
     * Don't trust this method. It doesn't seem
     * to have a clear-cut equivalent in the new
     * API, so I'm using something similar.
     * There's also an int cast, so the mantissa
     * in the double is lost.
     *
     * @param Current limit in amps.
     */
    public void setCurrentLimit(double l)
    {
        if (mTalon != null)
        {
            mTalon.configPeakCurrentLimit((int) l, sUpdateTimeoutMS); // XXX: Is this actually equivalent?
        }
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

    public boolean isForwardSoftLimitEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eForwardSoftLimitEnable, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
        else
            return true;
    }

    public boolean isZeroSensorPosOnFwdLimitEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eClearPositionOnLimitF, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
        else
            return false;
    }

    public int getPulseWidthRiseToRiseUs()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getPulseWidthRiseToRiseUs();
        else
            return 0;
    }

    public double getError()
    {
        if (mTalon != null)
            return mTalon.getClosedLoopError(sPidIdx);
        else
            return 0.0;
    }

    // FIXME: I can't find how to do this in the new API.
    //  transition notes suggest something to do with getPulseWidthRiseToRiseUs()
    public boolean isSensorPresent(FeedbackDevice d)
    {
        return true;
    }

    // FIXME: What's the difference between isControlEnabled and isEnabled?
    public boolean isControlEnabled()
    {
        if (mTalon != null)
            return mTalon.getControlMode() != ControlMode.Disabled ? true : false;
        else
            return false;
    }

    public boolean isEnabled()
    {
        if (mTalon != null)
            return mTalon.getControlMode() != ControlMode.Disabled ? true : false;
        else
            return false;
    }

    public boolean isZeroSensorPosOnRevLimitEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eClearPositionOnLimitR, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
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

    public int getPulseWidthPosition()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getPulseWidthPosition();
        else
            return 0;
    }

    public boolean isZeroSensorPosOnIndexEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eClearPositionOnIdx, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
        else
            return false;
    }

    /**
     * Get motion magic cruise velocity.
     *
     * @return Velocity native units.
     */
    public double getMotionMagicCruiseVelocity()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eMotMag_VelCruise, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
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
    public boolean getFaultRevSoftLim()
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
    public boolean getFaultRevLim()
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
    public boolean getStickyFaultRevLim()
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
    public boolean getStickyFaultRevSoftLim()
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

    /**
     * Get the current encoder position in encoder native units.
     *
     * @return Encoder position.
     */
    public int getEncPosition()
    {
        if (mTalon != null)
        {
            return mTalon.getSelectedSensorPosition(sPidIdx);
        }
        else
            return 0;
    }

    /**
     * Get the analog position of the encoder, in volts?
     *
     * <b>Not backwards compatible</b>
     *
     * @return Analog position (probably volts?)
     */
    public double getAnalogInPosition()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eAnalogPosition, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
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
    public boolean getFaultUnderVoltage()
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
     * The current ramp rate in closed loop mode.
     *
     * @return Closed loop ramp rate.
     */
    public double getCloseLoopRampRate()
    {
        if (mTalon != null)
        {
            return mTalon.configGetParameter(ParamEnum.eClosedloopRamp, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        }
        else
            return 0.0;
    }

    /**
     * Get the position of the active trajectory.
     *
     * @return Position of the active trajectory.
     */
    public double getMotionMagicActTrajPosition()
    {
        if (mTalon != null)
            return mTalon.getActiveTrajectoryPosition();
        else
            return 0.0;
    }

    /**
     * Get the currently set proportional of the PID.
     *
     * @return Current proportional of the PID.
     */
    public double getP()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eProfileParamSlot_P, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * Get the currently set feedforward of the PID.
     *
     * @return Current proportional of the PID.
     */
    public double getF()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eProfileParamSlot_F, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    public int getAnalogInVelocity()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getAnalogInVel();
        else
            return 0;
    }

    /**
     * Get the currently set integral of the PID.
     *
     * @return Current derivative of the PID.
     */
    public double getI()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eProfileParamSlot_I, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * Sets the integral zone. Who knows what that is.
     *
     * @return The integral zone.
     */
    public double getIZone()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eProfileParamSlot_IZone, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * This checks if the soft limit (switch) that's
     * reversed is enabled? I don't know.
     *
     * @return Is the reverse(d?) soft limit (switch?) enabled
     */
    public boolean isReverseSoftLimitEnabled()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eReverseSoftLimitEnable, sDefaultOrdinal,
                    sUpdateTimeoutMS) == 1 ? true : false;
        else
            return false;
    }

    public int getEncVelocity()
    {
        if (mTalon != null)
            return mTalon.getSelectedSensorVelocity(sPidIdx);
        else
            return 0;
    }

    /**
     * I think this is how fast we can get velocity measurements, but that's
     * just because they use simmilar terminology in their old documentation
     * when referring to pwm pulses. There's very little to work off here.
     *
     * @return Velocity measurement speed?
     */
    public double getVelocityMeasurementPeriod()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eSampleVelocityPeriod, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * This appears to set the amount of time that the Talon is allowed to take
     * to make velocity measurements?
     *
     * @return
     */
    public double getVelocityMeasurementWindow()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eSampleVelocityWindow, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * This <i>currently</i> gets if reverse soft limit is enabled.
     * <b>I don't know if that's what it did in the
     * old API</b>, because the documentation of the
     * old API was crap.
     *
     * @return Is the forward soft limit (switch?) enabled
     */
    public double getReverseSoftLimit()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eReverseSoftLimitEnable, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * Get the currently set derivative of the PID.
     *
     * @return Current derivative of the PID.
     */
    public double getD()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eProfileParamSlot_D, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * This gets if forward soft limit is enabled.
     * I don't know if that's what it did in the
     * old API, because the documentation of the
     * old API was crap.
     *
     * @return Is the forward soft limit (switch?) enabled
     */
    public double getForwardSoftLimit()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eForwardSoftLimitEnable, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

    /**
     * <b>Not backwards compatible.</b>
     */
    public boolean getPinStateQuadIdx()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getPinStateQuadIdx();
        else
            return false;
    }

    public int getAnalogInRaw()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getAnalogInRaw();
        else
            return 0;
    }

    /**
     * Get sensor speed/velocity.
     *
     * The speed units will be in the sensor's native ticks per 100ms.
     * For analog sensors, 3.3V corresponds to 1023 units. So a speed
     * of 200 equates to ~0.645 dV per 100ms or 6.451 dV per second.
     * If this is an analog encoder, that likely means 1.9548 rotations
     * per sec. For quadrature encoders, each unit corresponds a quadrature
     * edge (4X). So a 250 count encoder will produce 1000 edge events
     * per rotation. An example speed of 200 would then equate to 20%
     * of a rotation per 100ms, or 10 rotations per second.
     *
     * @return Sensor speed in native units per 100ms.
     */
    public double getSpeed()
    {
        if (mTalon != null)
            return nativeVelocityToRpm(mTalon.getSelectedSensorVelocity(sPidIdx));
        else
            return 0;
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
    public boolean getFaultForLim()
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
    public boolean getStickyFaultForLim()
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
    public boolean getFaultForSoftLim()
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
    public boolean getStickyFaultForSoftLim()
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
     * Get the PID error, if we're in a closed loop control mode.
     *
     * @return The current PID error.
     */
    public int getClosedLoopError()
    {
        if (mTalon != null)
        {
            // This method appears to be mis-documented. The
            // @param part of the javadoc says slotIdx (PID
            // gain slot), which makes the most sense, so
            // I'm going with that.
            return mTalon.getClosedLoopError(sPidIdx);
        }
        else
            return 0;
    }

    /**
     * Get the last value passed to the {@link set} method.
     *
     * @return The last value passed to set.
     */
    public double getSetpoint()
    {
        return mLastSetpoint;
    }

    /**
     * Get the state of the forward limit switch.
     * <b>This might be buggy.</b>
     *
     * @return State of the forward limit switch.
     */
    public boolean isFwdLimitSwitchClosed()
    {
        if (mTalon != null)
        {
            return mTalon.getSensorCollection().isFwdLimitSwitchClosed();
        }
        else
            return false;
    }

    /**
     * <b>Not backwards compatible.</b>
     */
    public boolean getPinStateQuadA()
    {
        if (mTalon != null)
        {
            return mTalon.getSensorCollection().getPinStateQuadA();
        }
        else
            return false;
    }

    /**
     * <b>Not backwards compatible.</b>
     */
    public boolean getPinStateQuadB()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getPinStateQuadB();
        else
            return false;
    }

    /**
     * Gets the integral accumulation of the motor controller.
     *
     * @return Integral accumulation.
     */
    public double getIaccum()
    {
        if (mTalon != null)
            return mTalon.getIntegralAccumulator(sPidIdx);
        else
            return 0.0;
    }

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
    public boolean getFaultHardwareFailure()
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
     * Get if the brake is enabled when the motor
     * becomes neutral.
     *
     * @return If we brake during neutral.
     */
    public boolean getBrakeEnableDuringNeutral()
    {
        return mNeutralMode == NeutralMode.Brake ? true : false;
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
    public boolean getStickyFaultUnderVoltage()
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

    public int getPulseWidthVelocity()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getPulseWidthVelocity();
        else
            return 0;
    }

    /**
     * Gets position. If {@link configEncoderCodesPerRev} has been called,
     * then this will return position in absolute wheel rotations. If that
     * hasn't been called, this will return absolute native units (see below).
     *
     * When using analog sensors, 0 units corresponds to 0V, 1023 units
     * corresponds to 3.3V
     * When using an analog encoder (wrapping around 1023 to 0 is possible) the
     * units are still
     * 3.3V per 1023 units.
     *
     * @return Absolute position in raw sensor units or wheel rotations.
     */
    public double getPosition()
    {
        // When using analog sensors, 0 units corresponds to 0V, 1023 units corresponds to 3.3V
        // When using an analog encoder (wrapping around 1023 to 0 is possible) the units are still
        // 3.3V per 1023 units.
        if (mTalon != null)
        {
            return encoderCodesToRotations(mTalon.getSelectedSensorPosition(sPidIdx));
        }
        else
            return 0.0;
    }

    public int getPulseWidthRiseToFallUs()
    {
        if (mTalon != null)
            return mTalon.getSensorCollection().getPulseWidthRiseToFallUs();
        else
            return 0;
    }

    /**
     * Get the acceleration of the motion magic controller.
     * If units are configured its in RPM per second,
     * otherwise it's native units.
     *
     * @return Acceleration of the motion magic controller.
     */
    public double getMotionMagicAcceleration()
    {
        if (mTalon != null)
            return mTalon.configGetParameter(ParamEnum.eMotMag_Accel, sDefaultOrdinal,
                    sUpdateTimeoutMS);
        else
            return 0.0;
    }

}
