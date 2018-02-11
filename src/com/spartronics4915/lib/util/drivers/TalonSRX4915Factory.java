package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Creates CANTalon objects and configures all the parameters we care about to
 * factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonSRX4915Factory
{

    // Create a CANTalon with the default (out of the box) configuration.
    public static TalonSRX4915 createDefaultMotor(int id)
    {
        TalonSRX4915 talon = createTalon(id, TalonSRX4915.Config.kDefaultMotor);
        talon.setControlMode(ControlMode.PercentOutput);
        return talon;
    }

    public static TalonSRX4915 createDefaultDrive(int id)
    {
        TalonSRX4915 talon = createTalon(id, TalonSRX4915.Config.kDriveMotor);
        talon.setControlMode(ControlMode.PercentOutput);
        return talon;
    }

    public static TalonSRX4915 createDefaultSlave(int id, int masterId, boolean isInverted)
    {
        TalonSRX4915 talon = createTalon(id, TalonSRX4915.Config.kDriveFollowerMotor);
        talon.configFollower(masterId, isInverted);
        return talon;
    }

    public static TalonSRX4915 createTalon(int id, TalonSRX4915.Config config)
    {
        TalonSRX4915 talon = new TalonSRX4915(id, config);
        return talon;
    }

    /**
     * Run this on a fresh talon to produce good values for the defaults.
     */
    public static String getFullTalonInfo(TalonSRX4915 talon)
    {
        return talon.dumpState();
    }
}
