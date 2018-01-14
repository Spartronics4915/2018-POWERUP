package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

/* CANTalon is a portability interface intended to facilitate
 * porting from CTRE CANTalon 2017 libs to CTRE Phoenix 2018.
 * This abstraction is useful as a buffer between all the
 * bad/weird stuff that CTRE does. It allows us to document
 * better, and even implement new features (e.g Talon
 * compatibility with Synthesis).
 * 
 * NB: this should be seen as a stopgap measure pending full
 *  migration to the new API.
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

public class CANTalon extends WPI_TalonSRX
{

    static final int s_defaultTimeoutMS = 0; // 0 for no blocking. This is like the old behavior (I think).
    static final int s_pidIdx = 0; // We're unsure about what this does.
    static final int s_defaultOrdinal = 0; // This probably does something specific on certain ParamEnums

    ControlMode m_controlMode = ControlMode.Disabled;
    ControlMode m_lastControlMode = null;
    NeutralMode m_neutralMode;
    double m_lastSetpoint = 0;
    int m_pidSlot;
    int m_maxVolts = 12;
    int m_codesPerRevolution; // Encoder codes per revolution
    boolean m_outputReversed = false;

    public CANTalon(int deviceNumber)
    {
        super(deviceNumber);
    }

    public CANTalon(int deviceNumber, int controlPeriodMs)
    {
        super(deviceNumber);
        this.setControlFramePeriod(controlPeriodMs, s_defaultTimeoutMS);
    }

    public CANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs)
    {
        super(deviceNumber);
        this.setControlFramePeriod(controlPeriodMs, enablePeriodMs);
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
        if (m_codesPerRevolution == 0) return (int) Math.round(rpm);
        double rotationsPer100MS = (rpm / 60) / 10;
        return (int) Math.round(rotationsPer100MS * m_codesPerRevolution);
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
        if (m_codesPerRevolution == 0) return (int) Math.round(nativeVelocity);
        double nativeUnitsPerMinute = nativeVelocity * 10 * 60;
        return (int) Math.round(nativeUnitsPerMinute / m_codesPerRevolution);
    }
    
    /**
     * Converts encoder codes (encoder specific)
     * to wheel rotations.
     * <b>configEncoderCodesPerRev must have been called for
     * this to work!</b>
     * 
     * @param Absolute encoder codes (use {@link nativeVelocityToRpm} for non-absolute units)
     * @return Absolute wheel rotations
     */
    private double encoderCodesToRotations(double codes) {
        if (m_codesPerRevolution == 0) return codes;
        return codes / m_codesPerRevolution;
    }

    /**
     * Sets the output on the Talon, depending on the mode you're in.
     * This is basically the only thing the new API does better,
     * IMO. You need to maintain state with the old API, and this
     * preserves that behavior.
     * 
     * @param The output depending on the ControlMode you've set the motor to.
     */
    @Override
    public void set(double value)
    {
        // We've integrated LazyCANTalon into here
        if (value != m_lastSetpoint || m_controlMode != m_lastControlMode) {
            if (m_outputReversed) value *= -1;
            this.set(m_controlMode, value);
            m_lastSetpoint = value;
            m_lastControlMode = m_controlMode;
        }
    }

    public void changeControlMode(ControlMode m)
    {
        this.m_controlMode = m; // in SRX mode, set() requires controlmode
    }

    public void setControlMode(ControlMode m)
    {
        this.m_controlMode = m;
    }

    public void setFeedbackDevice(FeedbackDevice d)
    {
        this.configSelectedFeedbackSensor(d, s_pidIdx, s_defaultTimeoutMS);
        this.setSensorPhase(false);
    }

    public void configEncoderCodesPerRev(int cpr)
    {
        m_codesPerRevolution = cpr;
    }

    public void setEncPosition(int p)
    {
        this.getSensorCollection().setQuadraturePosition(p, s_defaultTimeoutMS);
    }

    public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile)
    {
        this.config_kP(profile, p, s_defaultTimeoutMS);
        this.config_kI(profile, i, s_defaultTimeoutMS);
        this.config_kD(profile, d, s_defaultTimeoutMS);
        this.config_kF(profile, f, s_defaultTimeoutMS);
        this.config_IntegralZone(profile, izone, s_defaultTimeoutMS);
        double newRampRate = m_maxVolts / closeLoopRampRate;
        this.configClosedloopRamp(newRampRate, s_defaultTimeoutMS);
        this.configOpenloopRamp(newRampRate, s_defaultTimeoutMS);
    }

    public void setMotionMagicAcceleration(double motMagicAccel)
    {
        this.configMotionAcceleration(rpmToNativeVelocity(motMagicAccel), s_defaultTimeoutMS);
    }

    public void setMotionMagicCruiseVelocity(double kDriveLowGearMaxVelocity)
    {
        this.configMotionCruiseVelocity(rpmToNativeVelocity(kDriveLowGearMaxVelocity), s_defaultTimeoutMS);
    }

    public void clearIAccum()
    {
        this.setIntegralAccumulator(0, s_pidIdx, s_defaultTimeoutMS);
    }

    public void clearMotionProfileHasUnderrun()
    {
        this.clearMotionProfileHasUnderrun(s_defaultTimeoutMS);
    }

    public void clearStickyFaults()
    {
        this.clearStickyFaults(s_defaultTimeoutMS);
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public void configMaxOutputVoltage(double maxV)
    {
        // XXX: Really? Is configPeakOutputVoltage equivalent?
    }

    /**
     * Configure the nominal output voltage allowed.
     * <b>Because of how the new api works, min is ignored!</b>
     * 
     * @param max
     * @param <b>min (completely ignored)</b>
     */
    public void configNominalOutputVoltage(double max, double min)
    {
        // XXX: This was changed to use percentages, and just negate the percentage for the min.
        // That means that min doesn't do anything.
        this.configNominalOutputForward(max / m_maxVolts, s_defaultTimeoutMS);
        this.configNominalOutputReverse(max / m_maxVolts, s_defaultTimeoutMS);
    }

    /**
     * Configure the peak output voltage allowed.
     * <b>Because of how the new api works, min is ignored!</b>
     * 
     * @param max
     * @param <b>min (completely ignored)</b>
     */
    public void configPeakOutputVoltage(double max, double min)
    {
        // XXX: This was changed to use percentages, and just negate the percentage for the min.
        // That means that min doesn't do anything.
        this.configPeakOutputForward(max / m_maxVolts, s_defaultTimeoutMS);
        this.configPeakOutputReverse(max / m_maxVolts, s_defaultTimeoutMS);
    }

    public void enableBrakeMode(boolean s)
    {
        if (s)
            this.neutralOutput();
    }

    public void enableCurrentLimit(boolean enable)
    {
        this.enableCurrentLimit(enable);
    }

    public void setCurrentLimit(int amps)
    {
        this.configPeakCurrentLimit(amps, s_defaultTimeoutMS);
    }

    /**
     * Reverses motor output.
     * 
     * @param Is inverted or not.
     */
    public void reverseOutput(boolean s)
    {
        m_outputReversed = s;
        // XXX: this.setInverted seems to cause a stack overflow... This works around that.
    }
    
    /**
     * Configures the soft limit threshold on the forward sensor.
     * 
     * <b>Not backwards compatible</b>
     * 
     * @param l Limit in raw sensor units.
     */
    public void setForwardSoftLimit(int l)
    {
        this.configForwardSoftLimitThreshold(l, s_defaultTimeoutMS);
    }

    /**
     * Configures the soft limit threshold on the reverse sensor.
     * 
     * <b>Not backwards compatible</b>
     * 
     * @param l Limit in raw sensor units.
     */
    public void setReverseSoftLimit(int l)
    {
        this.configReverseSoftLimitThreshold(l, s_defaultTimeoutMS);
    }

    public void setNominalClosedLoopVoltage(double v)
    {
        // XXX: These are now in percentages, not volts... And there's no closed-loop only method.
        this.configNominalOutputForward(v / m_maxVolts, s_defaultTimeoutMS);
        this.configNominalOutputReverse(v / m_maxVolts, s_defaultTimeoutMS);
    }

    public void setPosition(int d)
    {
        this.getSensorCollection().setAnalogPosition(d, s_defaultTimeoutMS);
    }

    public void setProfile(int p)
    {
        // Select which closed loop profile to use, and uses whatever PIDF gains and the such that are already there.
        m_pidSlot = p;
        this.selectProfileSlot(p, s_pidIdx);
    }

    public void setPulseWidthPosition(int p)
    {
        this.getSensorCollection().setPulseWidthPosition(p, s_defaultTimeoutMS);
    }

    public void setSafetyEnabled(boolean b)
    {
        this.setSafetyEnabled(b);
    }

    public void setVelocityMeasurementPeriod(VelocityMeasPeriod p)
    {
        this.configVelocityMeasurementPeriod(p, s_defaultTimeoutMS);
    }

    public void setVelocityMeasurementWindow(int w)
    {
        this.configVelocityMeasurementWindow(w, s_defaultTimeoutMS);
    }

    public void setVoltageCompensationRampRate(double rampRate)
    {
        this.configVoltageCompSaturation(rampRate, s_defaultTimeoutMS); // XXX: I have no idea if this is these are the right units.
    }

    /**
     * Set the voltage ramp rate.
     * <b>This is no longer in volts/second, now it's the minimum
     * desired time to go from neutral to full throttle</b>
     * 
     * @param rampRate
     */
    public void setVoltageRampRate(double rampRate)
    {
        double newRampRate = m_maxVolts / rampRate;
        this.configClosedloopRamp(newRampRate, s_defaultTimeoutMS);
        this.configOpenloopRamp(newRampRate, s_defaultTimeoutMS);
    }

    public void setStatusFrameRateMs(StatusFrameEnhanced statFrame, int rate)
    {
        this.setStatusFramePeriod(statFrame, rate, s_defaultTimeoutMS);
    }

    public void reverseSensor(boolean s)
    {
        this.setSensorPhase(s);
    }

    public void setAnalogPosition(int pos)
    {
        this.getSensorCollection().setAnalogPosition(pos, s_defaultTimeoutMS);
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
        this.configPeakCurrentLimit((int) l, s_defaultTimeoutMS); // XXX: Is this actually equivalent?
    }

    public void enableForwardSoftLimit(boolean s)
    {
        this.configForwardSoftLimitEnable(s, s_defaultTimeoutMS);
    }

    /**
     * There is no equivalent in the new api for this.
     * You have to enable all or none, so you
     * should use those methods.
     * 
     * @deprecated
     * @return void
     */
    public void enableLimitSwitch(boolean fwd, boolean rev)
    {
    }

    public void enableReverseSoftLimit(boolean s)
    {
        this.configReverseSoftLimitEnable(s, s_defaultTimeoutMS);
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public void enableZeroSensorPositionOnForwardLimit(boolean s)
    {
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public void enableZeroSensorPositionOnIndex(boolean fwd, boolean rev)
    {
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public void enableZeroSensorPositionOnReverseLimit(boolean s)
    {
    }

    public void configFwdLimitSwitchNormallyOpen(boolean s)
    {
        this.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, /*
                                                                               * XXX:
                                                                               * LimitSwitchSource
                                                                               */
                s ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed,
                s_defaultTimeoutMS);
    }

    public void configRevLimitSwitchNormallyOpen(boolean s)
    {
        this.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, /*
                                                                               * XXX:
                                                                               * LimitSwitchSource
                                                                               */
                s ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed,
                s_defaultTimeoutMS);
    }

    public boolean isRevLimitSwitchClosed()
    {
        return this.getSensorCollection().isRevLimitSwitchClosed();
    }

    public double getBusVoltage()
    {
        // We keep stubs like this around just in case we want to change the API.
        return super.getBusVoltage();
    }

    public boolean isForwardSoftLimitEnabled()
    {
        return this.configGetParameter(ParamEnum.eForwardSoftLimitEnable, s_defaultOrdinal, s_defaultTimeoutMS) == 1 ? true : false;
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public int getStickyFaultOverTemp()
    {
        return 0;
    }

    public boolean isZeroSensorPosOnFwdLimitEnabled()
    {
        return this.configGetParameter(ParamEnum.eClearPositionOnLimitF, s_defaultOrdinal, s_defaultTimeoutMS) == 1 ? true : false;
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public int getNumberOfQuadIdxRises()
    {
        return 0;
    }

    public int getPulseWidthRiseToRiseUs()
    {
        return this.getSensorCollection().getPulseWidthRiseToRiseUs();
    }

    public double getError()
    {
        return this.getClosedLoopError(s_pidIdx);
    }

    // FIXME: I can't find how to do this in the new API.
    public boolean isSensorPresent(FeedbackDevice d)
    {
        return true;
    }

    // FIXME: What's the difference between isControlEnabled and isEnabled?
    public boolean isControlEnabled()
    {
        return this.getControlMode() != ControlMode.Disabled ? true : false;
    }

    public boolean isEnabled()
    {
        return this.getControlMode() != ControlMode.Disabled ? true : false;
    }

    public boolean isZeroSensorPosOnRevLimitEnabled()
    {
        return this.configGetParameter(ParamEnum.eClearPositionOnLimitR, s_defaultOrdinal, s_defaultTimeoutMS) == 1 ? true : false;
    }

    public double getOutputVoltage()
    {
        return this.getMotorOutputVoltage();
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public void getSmartDashboardType()
    {
    }

    public int getPulseWidthPosition()
    {
        return this.getSensorCollection().getPulseWidthPosition();
    }

    public boolean isZeroSensorPosOnIndexEnabled()
    {
        return this.configGetParameter(ParamEnum.eClearPositionOnIdx, s_defaultOrdinal, s_defaultTimeoutMS) == 1 ? true : false;
    }

    /**
     * Get motion magic cruise velocity.
     * 
     * @return Velocity native units.
     */
    public double getMotionMagicCruiseVelocity()
    {
        return this.configGetParameter(ParamEnum.eMotMag_VelCruise, s_defaultOrdinal, s_defaultTimeoutMS);
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
        Faults faults = new Faults();
        this.getFaults(faults);
        return faults.ForwardLimitSwitch;
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
        Faults faults = new Faults();
        this.getFaults(faults);
        return faults.ReverseLimitSwitch;
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
        StickyFaults faults = new StickyFaults();
        this.getStickyFaults(faults);
        return faults.ReverseLimitSwitch;
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
        StickyFaults faults = new StickyFaults();
        this.getStickyFaults(faults);
        return faults.ReverseSoftLimit;
    }

    /**
     * Get the current encoder position in encoder native units.
     * 
     * @return Encoder position.
     */
    public int getEncPosition()
    {
        return this.getSelectedSensorPosition(s_pidIdx);
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
        return this.configGetParameter(ParamEnum.eAnalogPosition, s_defaultOrdinal, s_defaultTimeoutMS);
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
        Faults faults = new Faults();
        this.getFaults(faults);
        return faults.UnderVoltage;
    }

    /**
     * The current ramp rate in closed loop mode.
     * 
     * @return Closed loop ramp rate.
     */
    public double getCloseLoopRampRate()
    {
        return this.configGetParameter(ParamEnum.eClosedloopRamp, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * Get the position of the active trajectory.
     * 
     * @return Position of the active trajectory.
     */
    public double getMotionMagicActTrajPosition()
    {
        return this.getActiveTrajectoryPosition();
    }

    /**
     * Get the currently set proportional of the PID.
     * 
     * @return Current proportional of the PID.
     */
    public double getP()
    {
        return this.configGetParameter(ParamEnum.eProfileParamSlot_P, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * Get the currently set feedforward of the PID.
     * 
     * @return Current proportional of the PID.
     */
    public double getF()
    {
        return this.configGetParameter(ParamEnum.eProfileParamSlot_F, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    public int getAnalogInVelocity()
    {
        return this.getSensorCollection().getAnalogInVel();
    }

    /**
     * Get the currently set integral of the PID.
     * 
     * @return Current derivative of the PID.
     */
    public double getI()
    {
        return this.configGetParameter(ParamEnum.eProfileParamSlot_I, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * Sets the integral zone. Who knows what that is.
     * 
     * @return The integral zone.
     */
    public double getIZone()
    {
        return this.configGetParameter(ParamEnum.eProfileParamSlot_IZone, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * This checks if the soft limit (switch) that's
     * reversed is enabled? I don't know.
     * 
     * @return Is the reverse(d?) soft limit (switch?) enabled
     */
    public boolean isReverseSoftLimitEnabled()
    {
        return this.configGetParameter(ParamEnum.eReverseSoftLimitEnable, s_defaultOrdinal, s_defaultTimeoutMS) == 1 ? true : false;
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return void
     */
    public void getPIDSourceType()
    {
        // not implemented - not available in 2018
    }

    public int getEncVelocity()
    {
        return this.getSelectedSensorVelocity(s_pidIdx);
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
        return this.configGetParameter(ParamEnum.eSampleVelocityPeriod, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * This appears to set the amount of time that the Talon is allowed to take
     * to make velocity measurements?
     * 
     * @return
     */
    public double getVelocityMeasurementWindow()
    {
        return this.configGetParameter(ParamEnum.eSampleVelocityWindow, s_defaultOrdinal, s_defaultTimeoutMS);
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
        return this.configGetParameter(ParamEnum.eReverseSoftLimitEnable, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * Get the currently set derivative of the PID.
     * 
     * @return Current derivative of the PID.
     */
    public double getD()
    {
        return this.configGetParameter(ParamEnum.eProfileParamSlot_D, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return 0
     */
    public boolean getFaultOverTemp()
    {
        return false;
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
        return this.configGetParameter(ParamEnum.eForwardSoftLimitEnable, s_defaultOrdinal, s_defaultTimeoutMS);
    }

    /**
     * <b>Not backwards compatible.</b>
     */
    public boolean getPinStateQuadIdx()
    {
        return this.getSensorCollection().getPinStateQuadIdx();
    }

    public int getAnalogInRaw()
    {
        return this.getSensorCollection().getAnalogInRaw();
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
        return nativeVelocityToRpm(this.getSelectedSensorVelocity(s_pidIdx));
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
        Faults faults = new Faults();
        this.getFaults(faults);
        return faults.ForwardLimitSwitch;
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
        StickyFaults faults = new StickyFaults();
        this.getStickyFaults(faults);
        return faults.ForwardLimitSwitch;
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
        Faults faults = new Faults();
        this.getFaults(faults);
        return faults.ForwardSoftLimit;
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
        StickyFaults faults = new StickyFaults();
        this.getStickyFaults(faults);
        return faults.ForwardSoftLimit;
    }

    /**
     * Get the PID error, if we're in a closed loop control mode.
     * 
     * @return The current PID error.
     */
    public int getClosedLoopError()
    {
        // This method appears to be mis-documented. The 
        // @param part of the javadoc says slotIdx (PID
        // gain slot), which makes the most sense, so
        // I'm going with that.
        return this.getClosedLoopError(m_pidSlot);
    }

    /**
     * Get the last value passed to the {@link set} method.
     * 
     * @return The last value passed to set.
     */
    public double getSetpoint()
    {
        return m_lastSetpoint;
    }

    /**
     * Get the state of the forward limit switch.
     * <b>This might be buggy.</b>
     * 
     * @return State of the forward limit switch.
     */
    public boolean isFwdLimitSwitchClosed()
    {
        return this.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * <b>Not backwards compatible.</b>
     */
    public boolean getPinStateQuadA()
    {
        return this.getSensorCollection().getPinStateQuadA();
    }

    /**
     * <b>Not backwards compatible.</b>
     */
    public boolean getPinStateQuadB()
    {
        return this.getSensorCollection().getPinStateQuadB();
    }

    /**
     * Gets the integral accumulation of the motor controller.
     * 
     * @return Integral accumulation.
     */
    public double getIaccum()
    {
        return this.getIntegralAccumulator(s_pidIdx);
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
        Faults faults = new Faults();
        this.getFaults(faults);
        return faults.HardwareFailure;
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return 0
     */
    public double pidGet() // wpilib PIDSource
    {
        return 0.;
    }

    /**
     * Get if the brake is enabled when the motor
     * becomes neutral.
     * 
     * @return If we brake during neutral.
     */
    public boolean getBrakeEnableDuringNeutral()
    {
        return m_neutralMode == NeutralMode.Brake ? true : false;
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
        StickyFaults faults = new StickyFaults();
        this.getStickyFaults(faults);
        return faults.UnderVoltage;
    }

    public int getPulseWidthVelocity()
    {
        return this.getSensorCollection().getPulseWidthVelocity();
    }

    /**
     * There is no equivalent in the new api for this.
     * 
     * @deprecated
     * @return 0
     */
    public double getNominalClosedLoopVoltage()
    {
        // the currently selected nominal closed loop voltage. Zero (Default) means feature is disabled.
        return 0.;
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
        return encoderCodesToRotations(this.getSelectedSensorPosition(s_pidIdx));
    }

    public int getPulseWidthRiseToFallUs()
    {
        return this.getSensorCollection().getPulseWidthRiseToFallUs();
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
        return this.configGetParameter(ParamEnum.eMotMag_Accel, s_defaultOrdinal, s_defaultTimeoutMS);
    }
}
