package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * CANTalon4915Drive encapsulates a 'standard' 4 CIM drive with IMU.
 * We currently assume that there are encoders on the master
 * motors and that these are speced & mounted following 4915
 * conventions.
 *
 * Future plans
 * - add support for low/high gear shifter
 **/
public class TalonSRX4915Drive
{

    public enum Config
    {
        kLeftNormalRightInverted(0),
        kRightNormalLeftInverted(1);

        public final int mValue;

        Config(int initValue)
        {
            mValue = initValue;
        }
    };

    private final int mQuadCodesPerRev;

    private TalonSRX4915 mLeftMaster;
    private TalonSRX4915 mLeftSlave;
    private TalonSRX4915 mRightMaster;
    private TalonSRX4915 mRightSlave;
    private PigeonIMU mIMU;
    private Config mConfig;
    private boolean mInitialized;
    private double mWheelDiameterInches;

    public TalonSRX4915Drive(double wheelDiameterInches,
            int encoderCodesPerRev,
            int leftMasterId, int leftSlaveId,
            int rightMasterId, int rightSlaveId,
            int pigeonHostId, // set the connected motor or -1
            // assuming we have encoders on master
            // we pigeon should be connected to
            // a slave
            Config c)
    {
        boolean invertLeft, invertRight;
        TalonSRX4915 pigeonTalon = null;
        mConfig = c;
        if (mConfig == Config.kLeftNormalRightInverted)
        {
            invertLeft = false;
            invertRight = true;
        }
        else
        {
            invertLeft = true;
            invertRight = false;
        }
        mQuadCodesPerRev = encoderCodesPerRev * 4;
        mWheelDiameterInches = wheelDiameterInches;
        mLeftMaster = TalonSRX4915Factory.createDefaultDrive(leftMasterId);
        mLeftMaster.configMotorAndSensor(invertLeft,
                FeedbackDevice.QuadEncoder, invertLeft/* phase */,
                mQuadCodesPerRev);
        mLeftSlave = TalonSRX4915Factory.createDefaultSlave(leftSlaveId, leftMasterId,
                invertLeft);
        if (pigeonHostId == leftSlaveId)
            pigeonTalon = mLeftSlave;

        mRightMaster = TalonSRX4915Factory.createDefaultDrive(rightMasterId);
        mRightMaster.configMotorAndSensor(invertRight,
                FeedbackDevice.QuadEncoder, invertRight/* phase */,
                mQuadCodesPerRev);
        mRightSlave = TalonSRX4915Factory.createDefaultSlave(rightSlaveId, rightMasterId,
                invertRight);
        if (pigeonHostId == rightSlaveId)
            pigeonTalon = mRightSlave;

        if (mLeftMaster != null && mLeftSlave != null &&
                mRightMaster != null && mRightSlave != null)
        {
            mInitialized = true;
            if (pigeonTalon != null)
            {
                mIMU = new PigeonIMU(pigeonTalon.getTalon());
                if (mIMU.getState() == PigeonState.NoComm)
                {
                    mIMU = null; // caller should report error checking hasIMU
                }
            }
        }
        else
            mInitialized = false;
    }

    public boolean isInitialized()
    {
        return mInitialized;
    }

    public boolean hasIMU()
    {
        return (mIMU != null);
    }

    /* drive methods (typically called via looper) -------------------------- */
    public void stop()
    {
        mLeftMaster.stopMotor();
        mRightMaster.stopMotor();
    }

    public void beginOpenLoop(double rampRate,
            double nominalOutput, double peakOutput)
    {
        mLeftMaster.configOutputPower(true, /* isOpenLoop */
                rampRate,
                nominalOutput, peakOutput, // fwd
                -nominalOutput, -peakOutput // rev
        );
        mRightMaster.configOutputPower(true, /* isOpenLoop */
                rampRate,
                nominalOutput, peakOutput, // fwd
                -nominalOutput, -peakOutput // rev
        );
        mLeftSlave.configOutputPower(true, /* isOpenLoop */
                rampRate,
                nominalOutput, peakOutput, // fwd
                -nominalOutput, -peakOutput // rev
        );
        mRightSlave.configOutputPower(true, /* isOpenLoop */
                rampRate,
                nominalOutput, peakOutput, // fwd
                -nominalOutput, -peakOutput // rev
        );
        mLeftMaster.set(ControlMode.PercentOutput, 0.0);
        mRightMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void driveOpenLoop(double left, double right)
    {
        mLeftMaster.set(ControlMode.PercentOutput, left);
        mRightMaster.set(ControlMode.PercentOutput, right);
    }

    public void beginClosedLoopVelocity(int slotIdx, double nominalOutput)
    {
        mLeftMaster.setControlMode(ControlMode.Velocity);
        mLeftMaster.selectProfileSlot(slotIdx);
        mLeftMaster.configNominalOutput(nominalOutput, -nominalOutput);

        mRightMaster.setControlMode(ControlMode.Velocity);
        mRightMaster.selectProfileSlot(slotIdx);
        mRightMaster.configNominalOutput(nominalOutput, -nominalOutput);
    }

    public void driveVelocityInchesPerSec(double left, double right)
    {
        mLeftMaster.setVelocityRPM(inchesPerSecondToRpm(left));
        mRightMaster.setVelocityRPM(inchesPerSecondToRpm(right));
    }

    public void beginClosedLoopPosition(int slotIdx, double nominalOutput,
            double maxVelocityRPM, double maxAccelRPMPerSec)
    {
        mLeftMaster.setControlMode(ControlMode.MotionMagic);
        mLeftMaster.selectProfileSlot(slotIdx);
        mLeftMaster.configNominalOutput(nominalOutput, -nominalOutput);
        mLeftMaster.configMotionMagicRPM(maxVelocityRPM, maxAccelRPMPerSec);

        mRightMaster.setControlMode(ControlMode.MotionMagic);
        mRightMaster.selectProfileSlot(slotIdx);
        mRightMaster.configNominalOutput(nominalOutput, -nominalOutput);
        mRightMaster.configMotionMagicRPM(maxVelocityRPM, maxAccelRPMPerSec);
    }

    public void drivePositionInches(double left, double right)
    {
        mLeftMaster.setPositionRotations(inchesToRotations(left));
        mRightMaster.setPositionRotations(inchesToRotations(right));
    }

    /* distance and speed conversions ----------------------------------- */
    public double getLeftDistanceInches()
    {
        if (!isInitialized())
            return 0.0;
        return rotationsToInches(mLeftMaster.getSensorPositionRotations());
    }

    public double getRightDistanceInches()
    {
        if (!isInitialized())
            return 0.0;
        return rotationsToInches(mRightMaster.getSensorPositionRotations());
    }

    public double getLeftVelocityInchesPerSec()
    {
        if (!isInitialized())
            return 0.0;
        return rpmToInchesPerSecond(mLeftMaster.getSensorVelocityRPM());
    }

    public double getRightVelocityInchesPerSec()
    {
        if (!isInitialized())
            return 0.0;
        return rpmToInchesPerSecond(mRightMaster.getSensorVelocityRPM());
    }

    private double rotationsToInches(double rotations)
    {
        return rotations * (mWheelDiameterInches * Math.PI);
    }

    private double rpmToInchesPerSecond(double rpm)
    {
        return rotationsToInches(rpm) / 60;
    }

    private double inchesToRotations(double inches)
    {
        return inches / (mWheelDiameterInches * Math.PI);
    }

    private double inchesPerSecondToRpm(double inchesPerSec)
    {
        return inchesToRotations(inchesPerSec) * 60;
    }

    /* support/misc ------------------------------------------------------- */
    public double getGyroAngle()
    {
        // nb: this routine is called a lot (smartdashboard updates, etc)
        if (mIMU == null)
            return 0.0;
        double[] ypr = new double[3];
        mIMU.getYawPitchRoll(ypr);
        return ypr[0]; // degrees
    }

    public void setGyroAngle(double yawDegrees)
    {
        if (mIMU == null)
            return;
        mIMU.setYaw(yawDegrees, 0); // nonblocking
    }

    public void resetEncoders(boolean resetYaw)
    {
        mLeftMaster.resetSensor();
        mRightMaster.resetSensor();
        if (mIMU != null && resetYaw)
            mIMU.setYaw(0, 0); // nonblocking
    }

    public boolean isBrakingEnabled()
    {
        return mLeftMaster.isBrakeEnabled();
    }

    public void enableBraking(boolean onoff)
    {
        mLeftMaster.setBrakeMode(onoff);
        mRightMaster.setBrakeMode(onoff);

        mLeftSlave.setBrakeMode(onoff);
        mRightSlave.setBrakeMode(onoff);
    }

    public void reloadGains(int slotIdx, double kp, double ki, double kd, double kf,
            int izone, double rampRate)
    {
        mLeftMaster.configPID(slotIdx, kp, ki, kd, kf, izone, rampRate);
        mRightMaster.configPID(slotIdx, kp, ki, kd, kf, izone, rampRate);
    }

    public void outputToSmartDashboard(boolean usesVelocityControl)
    {
        final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
        
        SmartDashboard.putNumber("Drive/left voltage (V)", mLeftMaster.getOutputVoltage());
        SmartDashboard.putNumber("Drive/right voltage (V)", mRightMaster.getOutputVoltage());
        SmartDashboard.putNumber("Drive/left speed (ips)", left_speed);
        SmartDashboard.putNumber("Drive/right speed (ips)", right_speed);
        if (usesVelocityControl)
        {
            SmartDashboard.putNumber("Drive/left speed error (ips)",
                    rpmToInchesPerSecond(mLeftMaster.getSetpointRPM()) - left_speed);
            SmartDashboard.putNumber("Drive/right speed error (ips)",
                    rpmToInchesPerSecond(mRightMaster.getSetpointRPM()) - right_speed);
        }
        else
        {
            SmartDashboard.putNumber("Drive/left speed error (ips)", 0.0);
            SmartDashboard.putNumber("Drive/right speed error (ips)", 0.0);
        }
        SmartDashboard.putNumber("Drive/left position (rotations)",
                mLeftMaster.getSetpointRotations());
        SmartDashboard.putNumber("Drive/right position (rotations)",
                mRightMaster.getSetpointRotations());
        // following names relied upon by dashboard.
        SmartDashboard.putNumber("Drivetrain_IMU_Heading", getGyroAngle());
    }

    public boolean checkSystem()
    {
        logNotice("checkSystem() ---------------------------------");
        final double kCurrentThres = 9;
        final double kRpmThres = 85;
        
        // first validate open-loop driving of all motors...
        this.beginOpenLoop(.5, 0.0, 1.0); // ramp, nominal, peak
        this.enableBraking(true);
        
        logNotice("rightMaster:\n" + mRightMaster.dumpState());
        logNotice("leftMaster:\n" + mLeftMaster.dumpState());
        logNotice("rightSlave:\n" + mRightSlave.dumpState());
        logNotice("leftSlave:\n" + mLeftSlave.dumpState());
        
        logNotice("Forward half power, 3 sec");
        this.driveOpenLoop(.5, .5); // <---------------
        Timer.delay(3.0);
        final double rpmFwdL = mLeftMaster.getSensorVelocityRPM();
        final double rpmFwdR = mRightMaster.getSensorVelocityRPM();
        this.driveOpenLoop(0, 0);
        this.resetEncoders(false);
        Timer.delay(2.0);
        
        logNotice("Reverse half power, 3 sec");
        this.driveOpenLoop(-.5, -.5); // <--------------
        Timer.delay(3.0);
        final double rpmRevL = mLeftMaster.getSensorVelocityRPM();
        final double rpmRevR = mRightMaster.getSensorVelocityRPM();
        this.driveOpenLoop(0, 0);
        Timer.delay(2.0);

        // disable followers, but retain current sense of inversion
        mRightMaster.setControlMode(ControlMode.PercentOutput);
        mRightSlave.setControlMode(ControlMode.PercentOutput);
        mLeftMaster.setControlMode(ControlMode.PercentOutput);
        mLeftSlave.setControlMode(ControlMode.PercentOutput);

        logNotice("rightMaster .5, 4 seconds");
        mRightMaster.set(.5); // <---------------------
        Timer.delay(4.0);
        final double currentRightMaster = mRightMaster.getOutputCurrent();
        final double rpmRightMaster = mRightMaster.getSensorVelocityRPM();
        mRightMaster.set(0.0f);

        Timer.delay(2.0);

        logNotice("rightSlave .5, 4 seconds");
        mRightSlave.set(.5); // <---------------------
        Timer.delay(4.0);
        final double currentRightSlave = mRightSlave.getOutputCurrent();
        final double rpmRightSlave = mRightMaster.getSensorVelocityRPM(); // only master has sensor
        mRightSlave.set(0.0f);

        Timer.delay(2.0);

        logNotice("leftMaster .5, 4 seconds");
        mLeftMaster.set(.5); // <--------------------
        Timer.delay(4.0);
        final double currentLeftMaster = mLeftMaster.getOutputCurrent();
        final double rpmLeftMaster = mLeftMaster.getSensorVelocityRPM();
        mLeftMaster.set(0.0f);

        Timer.delay(2.0);

        logNotice("leftSlave .5, 4 seconds");
        mLeftSlave.set(.5); // <-------------------
        Timer.delay(4.0);
        final double currentLeftSlave = mLeftSlave.getOutputCurrent();
        final double rpmLeftSlave = mLeftMaster.getSensorVelocityRPM(); // nb only master has sensor
        mLeftSlave.set(0.0);

        // restore master/slaves ----------------------------
        mRightMaster.setControlMode(ControlMode.PercentOutput);
        mLeftMaster.setControlMode(ControlMode.PercentOutput);
        mRightSlave.setControlMode(ControlMode.Follower);
        mRightSlave.set(mRightMaster.getId());
        mLeftSlave.setControlMode(ControlMode.Follower);
        mLeftSlave.set(mLeftMaster.getId());
        
        logNotice("RPM FWD L/R: " + rpmFwdL + "/" + rpmFwdR);
        logNotice("IPS FWD L/R: " + rpmToInchesPerSecond(rpmFwdL) + "/" +
                rpmToInchesPerSecond(rpmFwdR));
        
        logNotice("RPM REV L/R: " + rpmRevL + "/" + rpmRevR);
        logNotice("IPS REV L/R: " + rpmToInchesPerSecond(rpmRevL) + "/" +
                rpmToInchesPerSecond(rpmRevR));

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

    private void logNotice(String msg)
    {
        Logger.warning("CANTalonDrive " + msg);
    }

    private void logWarning(String msg)
    {
        Logger.warning("CANTalonDrive " + msg);
    }
}
