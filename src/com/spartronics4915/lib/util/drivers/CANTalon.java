package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


/* CANTalon is a portability interface intended to facilitate
 * porting from CTRE CANTalon 2017 libs to CTRE Phoenix 2018
 * NB: this should be seen as a stopgap measure pending full
 *  migration to the new API.
 */

public class CANTalon extends WPI_TalonSRX
{
    ControlMode m_controlMode = ControlMode.Disabled;
    static final int s_defaultTimeoutMS = 5;
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
        this.configSelectedFeedbackSensor(d, 0/*XXX:pidIdx*/, s_defaultTimeoutMS);
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
        this.setIntegralAccumulator(0, 0/*XXX:pidIdx*/, s_defaultTimeoutMS/*timeoutMs*/);
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
    public void setForwardSoftLimit(double l)
    {
    }
    public void setReverseSoftLimit(double l)
    {
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
    public int getFaultOverTemp()
    {
        return 0;
    }
    public int getForwardSoftLimit()
    {
        return 0;
    }
    public long GetFirmwareVersion()
    {
        return 0;
    }
    public int getPinStateQuadIdx()
    {
        return 0;
    }
    public int getAnalogInRaw()
    {
        return 0;
    }
    public int getFaultForLim()
    {
        return 0;
    }
    public double getSpeed()
    {
        return 0.;
    }
    public int getStickyFaultForLim()
    {
        return 0;
    }
    public int getFaultForSoftLim()
    {
        return 0;
    }
    public int getStickyFaultForSoftLim()
    {
        return 0;
    }
    public int getClosedLoopError()
    {
        return 0; // Get the current difference between the setpoint and the sensor value.
    }
    public double getSetpoint()
    {
        return 0.;
    }
    public boolean isFwdLimitSwitchClosed()
    {
        return false;
    }
    public int getPinStateQuadA()
    {
        return 0;
    }
    public int getPinStateQuadB()
    {
        return 0;
    }
    public long GetIaccum()
    {
        return 0;
    }
    public int getFaultHardwareFailure()
    {
        return 0;
    }
    public double pidGet() // wpilib PIDSource
    {
        return 0.;
    }
    public boolean getBrakeEnableDuringNeutral()
    {
        return false;
    }
    public int getStickyFaultUnderVoltage()
    {
        return 0;
    }
    public int getPulseWidthVelocity()
    {
        return 0;
    }
    public double GetNominalClosedLoopVoltage()
    {
        // the currently selected nominal closed loop voltage. Zero (Default) means feature is disabled.
        return 0.;
    }
    public double getPosition()
    {
        // When using analog sensors, 0 units corresponds to 0V, 1023 units corresponds to 3.3V 
        // When using an analog encoder (wrapping around 1023 to 0 is possible) the units are still 
        // 3.3V per 1023 units.
        return 0.;
    }
    public int getPulseWidthRiseToFallUs()
    {
        return 0;
    }
    public int getMotionMagicAcceleration()
    {
        return 0;
    }
}
