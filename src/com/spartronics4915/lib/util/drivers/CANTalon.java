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
    static final int s_defaultTimeoutMS = 5;
    static final int s_pidIdx = 0; // We're unsure about what this does.
    
    ControlMode m_controlMode = ControlMode.Disabled;
    NeutralMode m_neutralMode;
    double m_lastSetpoint = 0;
    int m_pidSlot;
    
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
     * Sets the output on the Talon, depending on the mode you're in.
     * This is basically the only thing the new API does better,
     * IMO. You need to maintain state with the old API, and this
     * preserves that behavior.
     */
    @Override
    public void set(double value) {
        m_lastSetpoint = value;
        super.set(m_controlMode, value);
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
    }
    public void setEncPosition(int p)
    {
    }
    public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile)
    {
    }
    public void setMotionMagicAcceleration(double motMagicAccel)
    {
    }
    public void setMotionMagicCruiseVelocity(double motMagicCruiseVeloc)
    {
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
    public void configMaxOutputVoltage(double maxV)
    {
    }
    public void configNominalOutputVoltage(double max, double min)
    {
    }
    public void configPeakOutputVoltage(double max, double min)
    {
    }
    public void enableBrakeMode(boolean s)
    {
    }
    public void enableCurrentLimit(double max)
    {
    }
    public void reverseOutput(boolean s)
    {
    }
    /**
     * Configures the soft limit threshold on the forward sensor.
     * 
     * <b>Not backwards compatible</b>
     * @param l Limit in raw sensor units.
     */
    public void setForwardSoftLimit(int l)
    {
        this.configForwardSoftLimitThreshold(l, 0);
    }
    /**
     * Configures the soft limit threshold on the reverse sensor.
     * 
     * <b>Not backwards compatible</b>
     * @param l Limit in raw sensor units.
     */
    public void setReverseSoftLimit(int l)
    {
        this.configReverseSoftLimitThreshold(l, 0);
    }
    public void setInverted(boolean s)
    {
    }
    public void setNominalClosedLoopVoltage(double v)
    {
    }
    public void setPosition(double d)
    {
    }
    public void setProfile(int p)
    {
        // Select which closed loop profile to use, and uses whatever PIDF gains and the such that are already there.
    }
    public void setPulseWidthPosition(int p)
    {
    }
    public void setSafetyEnabled(boolean b)
    {
    }
    public void SetVelocityMeasurementPeriod(VelocityMeasPeriod p)
    {
    }
    public void SetVelocityMeasurementWindow(int w)
    {
    }
    public void setVoltageCompensationRampRate(double rampRate)
    {
    }
    public void setVoltageRampRate(double rampRate)
    {
    }
    public void setStatusFrameRateMs(StatusFrameEnhanced statFrame, int rate)
    {
    }
    public void reverseSensor(boolean s)
    {
    }
    public void setAnalogPosition(int pos)
    {
    }
    public void setCurrentLimit(double l)
    {
    }
    public void enableForwardSoftLimit(boolean s)
    {
    }
    public void enableLimitSwitch(boolean fwd, boolean rev)
    {
    }
    public void enableReverseSoftLimit(boolean s)
    {
    }
    public void enableZeroSensorPositionOnForwardLimit(boolean s)
    {
    }
    public void enableZeroSensorPositionOnIndex(boolean fwd, boolean rev)
    {
    }
    public void enableZeroSensorPositionOnReverseLimit(boolean s)
    {
    }
    public void configFwdLimitSwitchNormallyOpen(boolean s)
    {
        this.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, /* XXX: LimitSwitchSource */
                                    s ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed,
                                    s_defaultTimeoutMS);
    }
    public void configRevLimitSwitchNormallyOpen(boolean s)
    {
        this.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, /* XXX: LimitSwitchSource */
                                    s ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed,
                                    s_defaultTimeoutMS);
    }
    public boolean isRevLimitSwitchClosed()
    {
        return true;
    }
    public double getBusVoltage()
    {
        return 0.;
    }
    public boolean isForwardSoftLimitEnabled()
    {
        return true;
    }
    public double getFaultRevSoftLim()
    {
        return 0.;
    }
    public int getStickyFaultOverTemp()
    {
        return 0;
    }
    public boolean isZeroSensorPosOnFwdLimitEnabled()
    {
       return true;
    }
    public int getNumberOfQuadIdxRises()
    {
        return 0;
    }
    public int getPulseWidthRiseToRiseUs()
    {
        return 0;
    }
    public double getError()
    {
        return 0.;
    }
    public boolean isSensorPresent(FeedbackDevice d)
    {
        return true;
    }
    public boolean isControlEnabled()
    {
        return true;
    }
    public boolean isEnabled()
    {
        return true; /* XXX */
    }
    public boolean isZeroSensorPosOnRevLimitEnabled()
    {
        return true; // XXX
    }
    public double getOutputVoltage()
    {
        return 0.;
    }
    public void getSmartDashboardType()
    {
    }
    public int getPulseWidthPosition()
    {
       return 0;
    }
    public boolean isZeroSensorPosOnIndexEnabled()
    {
        return true;
    }
    public double getMotionMagicCruiseVelocity()
    {
        return 0.;
    }
    public int getStickyFaultRevSoftLim()
    {
        return 0;
    }
    public int getFaultRevLim()
    {
        return 0;
    }
    public int getEncPosition()
    {
        return 0;
    }
    public int getAnalogInPosition()
    {
        return 0;
    }
    public int getFaultUnderVoltage()
    {
        return 0;
    }
    public double getCloseLoopRampRate()
    {
        return 0.;
    }
    public double getMotionMagicActTrajPosition()
    {
        return 0.;
    }
    public double getP()
    {
        return 0.; // XXX
    }
    public double getF()
    {
        return 0.; // XXX
    }
    public int getAnalogInVelocity()
    {
        return 0;
    }
    public double getI()
    {
       return 0.; // XXX
    }
    public double getIZone()
    {
        return 0.;
    }
    public boolean isReverseSoftLimitEnabled()
    {
        return true;
    }
    public void getPIDSourceType()
    {
        // not implemented - not available in 2018
    }
    public int getEncVelocity()
    {
        return 0;
    }
    public VelocityMeasPeriod GetVelocityMeasurementPeriod()
    {
        return VelocityMeasPeriod.Period_1Ms; // FIXME
    }
    public int GetVelocityMeasurementWindow()
    {
        return 0;
    }
    public boolean getStickyFaultRevLim()
    {
        return true;
    }
    public int getReverseSoftLimit()
    {
        return 0;
    }
    public double getD()
    {
        return 0.;
    }
    /**
     * There is no equivalent in the new api for this.
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
    public int getForwardSoftLimit()
    {
        return super.configGetCustomParam(ParamEnum.eForwardSoftLimitEnable.value, s_defaultTimeoutMS); // Why does this take an int instead of an enum? Who knows!
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public int getPinStateQuadIdx()
    {
        return 0;
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public int getAnalogInRaw()
    {
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
        return super.getSelectedSensorVelocity(s_pidIdx);
    }
    /**
     * Check if there's a forward limit switch issue.
     * 
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     * @return Is there a hardware failure.
     */
    public boolean getFaultForLimit()
    {
        Faults faults = new Faults();
        super.getFaults(faults);
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
     * @return Is there a limit switch issue.
     */
    public boolean getStickyFaultForLim()
    {
        StickyFaults faults = new StickyFaults();
        super.getStickyFaults(faults);
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
     * @return Is there a hardware failure.
     */
    public boolean getFaultForSoftLimit()
    {
        Faults faults = new Faults();
        super.getFaults(faults);
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
     * @return Is there a limit switch issue.
     */
    public boolean getStickyFaultForSoftLim()
    {
        StickyFaults faults = new StickyFaults();
        super.getStickyFaults(faults);
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
        return super.getClosedLoopError(m_pidSlot);
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
        return super.configGetCustomParam(ParamEnum.eForwardSoftLimitEnable.value, s_defaultTimeoutMS) == 1 ? true : false;
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public int getPinStateQuadA()
    {
        return 0;
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public int getPinStateQuadB()
    {
        return 0;
    }
    /**
     * Gets the integral accumulation of the motor controller.
     * 
     * @return Integral accumulation.
     */
    public double GetIaccum()
    {
        return super.getIntegralAccumulator(s_pidIdx);
    }
    /**
     * Check if there's a hardware failure.
     * 
     * In the old api this returned an int, but the new one only
     * returns a boolean. I suspect this is just more bad CTRE code, and
     * they should have used a boolean. It's also possible that
     * it meant something else before.
     * <b>Not backwards compatible</b>
     * @return Is there a hardware failure.
     */
    public boolean getFaultHardwareFailure()
    {
        Faults faults = new Faults();
        super.getFaults(faults);
        return faults.HardwareFailure;
    }
    /**
     * There is no equivalent in the new api for this.
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
     * @return Is under voltage or not.
     */
    public boolean getStickyFaultUnderVoltage()
    {
        StickyFaults faults = new StickyFaults();
        super.getStickyFaults(faults);
        return faults.UnderVoltage;
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public int getPulseWidthVelocity()
    {
        return 0;
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public double getNominalClosedLoopVoltage()
    {
        // the currently selected nominal closed loop voltage. Zero (Default) means feature is disabled.
        return 0.;
    }
    /**
     * Gets position.
     * 
     * When using analog sensors, 0 units corresponds to 0V, 1023 units corresponds to 3.3V 
     * When using an analog encoder (wrapping around 1023 to 0 is possible) the units are still 
     * 3.3V per 1023 units.
     * @return Position in raw sensor units.
     */
    public double getPosition()
    {
        // When using analog sensors, 0 units corresponds to 0V, 1023 units corresponds to 3.3V 
        // When using an analog encoder (wrapping around 1023 to 0 is possible) the units are still 
        // 3.3V per 1023 units.
        return super.getSelectedSensorPosition(s_pidIdx);
    }
    /**
     * There is no equivalent in the new api for this.
     * @deprecated
     * @return 0
     */
    public int getPulseWidthRiseToFallUs()
    {
        return 0;
    }
    /**
     * There is no equivalent method in the new API. This provides
     * a rough approximation, in raw sensor units. An integer cast
     * also happens, so this is truncated.
     * 
     * @return 
     */
    public int getMotionMagicAcceleration()
    {
        return super.configGetCustomParam(ParamEnum.eMotMag_Accel.value, s_defaultTimeoutMS); // Why does this take an int instead of an enum? Who knows!
    }
}
