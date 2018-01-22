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
 * CANTalon4915Drive ecapsulates a 'standard' 4 CIM drive with IMU.
 *      We currently assume that there are encoders on the master
 *      motors and that these are speced & mounted following 4915
 *      conventions.
 *
 *  Future plans
 *      - add support for low/high gear shifter
 **/
public class CANTalon4915Drive
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

    // for this encoder: E4P-250-250-N-S-D-D (western digital)
    //                        ^ is cpr, x 4 for quad
    private static final int kEncoderCodesPerRev = 1000;

    private CANTalon4915 mLeftMaster;
    private CANTalon4915 mLeftSlave;
    private CANTalon4915 mRightMaster;
    private CANTalon4915 mRightSlave;
    private PigeonIMU mIMU;
    private Config mConfig;
    private boolean mInitialized;
    private double mWheelDiameterInches;

    public CANTalon4915Drive(double wheelDiameterInches,
                            int leftMasterId, int leftSlaveId,
                            int rightMasterId, int rightSlaveId,
                            int pigeonHostId, // set the connected motor or -1
                                              // assuming we have encoders on master
                                              // we pigeon should be connected to
                                              // a slave
                            Config c)
    {
        boolean invertLeft, invertRight;
        CANTalon4915 pigeonTalon = null;
        mConfig = c;
        if(mConfig == Config.kLeftNormalRightInverted)
        {
            invertLeft = false;
            invertRight = true;
        }
        {
            invertLeft = true;
            invertRight = false;
        }
        mWheelDiameterInches = wheelDiameterInches;
        mLeftMaster = CANTalonFactory.createDefaultDrive(leftMasterId);
        mLeftMaster.configMotorAndEncoder(invertLeft,
                    FeedbackDevice.QuadEncoder, invertLeft/*phase*/,
                    kEncoderCodesPerRev);
        mLeftSlave = CANTalonFactory.createDefaultSlave(leftSlaveId, leftMasterId,
                    invertLeft);
        if(pigeonHostId == leftSlaveId)
            pigeonTalon = mLeftSlave;

        mRightMaster = CANTalonFactory.createDefaultDrive(rightMasterId);
        mRightMaster.configMotorAndEncoder(invertRight,
                    FeedbackDevice.QuadEncoder, invertRight/*phase*/,
                    kEncoderCodesPerRev);
        mRightSlave = CANTalonFactory.createDefaultSlave(rightSlaveId, rightMasterId,
                    invertRight);
        if(pigeonHostId == rightSlaveId)
            pigeonTalon = mRightSlave;

        if(mLeftMaster != null && mLeftSlave != null &&
           mRightMaster != null && mRightSlave != null)
        {
            mInitialized = true;
            if(pigeonTalon != null)
            {
                mIMU = new PigeonIMU(pigeonTalon.getTalon());
                if(mIMU.getState() == PigeonState.NoComm)
                    mIMU = null; // caller should report error checking hasIMU
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

    /* drive methods (typically called via looper) --------------------------*/
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
        driveVelocity(inchesPerSecondToRpm(left), inchesPerSecondToRpm(right));
    }
    
    private void driveVelocity(double left, double right)
    {
        mLeftMaster.set(ControlMode.Velocity, left);
        mRightMaster.set(ControlMode.Velocity, right);
    }
    
    public void beginClosedLoopPosition(int slotIdx, double nominalOutput,
                                    double maxVelocity, double maxAcceleration)
    {
        int maxV = rpmToSensorUnitsPer100ms(maxVelocity);
        int maxA = rpmPerSecToSensorUnitsPer100msPerSec(maxAcceleration);
        mLeftMaster.setControlMode(ControlMode.MotionMagic);
        mLeftMaster.selectProfileSlot(slotIdx);
        mLeftMaster.configNominalOutput(nominalOutput, -nominalOutput);
        mLeftMaster.configMotionMagic(maxV, maxA);
        
        mRightMaster.setControlMode(ControlMode.MotionMagic);
        mRightMaster.selectProfileSlot(slotIdx);
        mRightMaster.configNominalOutput(nominalOutput, -nominalOutput);
        mRightMaster.configMotionMagic(maxV, maxA);
    }
    
    public void drivePositionInches(double left, double right)
    {
        drivePosition(inchesToRotations(left), inchesToRotations(right));
    }
    
    private void drivePosition(double left, double right)
    {
        mLeftMaster.set(ControlMode.MotionMagic, left);
        mRightMaster.set(ControlMode.MotionMagic, right);
    }
    
    /* distance and speed conversions -----------------------------------*/
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
    
    private int rpmToSensorUnitsPer100ms(double maxRPM)
    {
        return mLeftMaster.rpmToNativeVelocity(maxRPM);
    }
    
    private int rpmPerSecToSensorUnitsPer100msPerSec(double maxRPMPerSec)
    {
        return mLeftMaster.rpmToNativeVelocity(maxRPMPerSec);
    }
    
    /* support/misc -------------------------------------------------------*/
    public double getGyroAngle()
    {
        if(mIMU == null) return 0.0;
        double[] ypr = new double[3];
        mIMU.getYawPitchRoll(ypr);
        return ypr[0]; // degrees
    }
    
    public void setGyroAngle(double yawDegrees)
    {
        if(mIMU == null) return;
        mIMU.setYaw(yawDegrees, 5);
    }

    public void resetEncoders(boolean resetYaw)
    {
        mLeftMaster.setEncPosition(0);
        mLeftMaster.setPosition(0);

        mRightMaster.setEncPosition(0);
        mRightMaster.setPosition(0);

        mLeftSlave.setPosition(0);
        mRightSlave.setPosition(0);

        if(mIMU != null && resetYaw)
            mIMU.setYaw(0, 5/*ms*/);
    }

    public void enableBraking(boolean onoff)
    {
        mLeftMaster.enableBrakeMode(onoff);
        mRightMaster.enableBrakeMode(onoff);

        mLeftSlave.enableBrakeMode(onoff);
        mRightSlave.enableBrakeMode(onoff);
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
        SmartDashboard.putNumber("left voltage (V)", mLeftMaster.getOutputVoltage());
        SmartDashboard.putNumber("right voltage (V)", mRightMaster.getOutputVoltage());
        SmartDashboard.putNumber("left speed (ips)", left_speed);
        SmartDashboard.putNumber("right speed (ips)", right_speed);
        if (usesVelocityControl)
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
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
        SmartDashboard.putNumber("Drivetrain_IMU_Heading", getGyroAngle());
    }
    
    public boolean checkSystem()
    {
        logNotice("checkSystem() ---------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 300;
        final double kMaxVoltage = 12.0;

        mRightMaster.setControlMode(ControlMode.PercentOutput); // was Voltage
        mRightSlave.setControlMode(ControlMode.PercentOutput);
        mLeftMaster.setControlMode(ControlMode.PercentOutput);
        mLeftSlave.setControlMode(ControlMode.PercentOutput);

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

        mRightMaster.setControlMode(ControlMode.PercentOutput);
        mLeftMaster.setControlMode(ControlMode.PercentOutput);

        mRightSlave.setControlMode(ControlMode.Follower);
        mRightSlave.set(mRightMaster.getId());

        mLeftSlave.setControlMode(ControlMode.Follower);
        mLeftSlave.set(mLeftMaster.getId());

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
    
    private void logError(String msg)
    {
        Logger.error("CANTalonDrive " + msg);
    }

}
