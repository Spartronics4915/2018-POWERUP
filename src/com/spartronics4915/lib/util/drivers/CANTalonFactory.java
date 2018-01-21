package com.spartronics4915.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Creates CANTalon objects and configures all the parameters we care about to
 * factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class CANTalonFactory
{
    // Create a CANTalon with the default (out of the box) configuration.
    public static CANTalon4915 createDefaultMotor(int id)
    {
        CANTalon4915 talon = createTalon(id, CANTalon4915.Config.kDefaultMotor);
        talon.setControlMode(ControlMode.PercentOutput);
        return talon;
    }
    
    public static CANTalon4915 createDefaultDrive(int id)
    {
        CANTalon4915 talon = createTalon(id, CANTalon4915.Config.kDriveMotor);
        talon.setControlMode(ControlMode.PercentOutput);
        return talon;
    }
    
    public static CANTalon4915 createDefaultSlave(int id, int masterId, boolean isInverted)
    {
        CANTalon4915 talon = createTalon(id, CANTalon4915.Config.kDriveFollowerMotor);
        talon.setControlMode(ControlMode.Follower);
        talon.set(masterId);
        talon.setInverted(isInverted);
        return talon;
    }
    
    public static CANTalon4915 createTalon(int id, CANTalon4915.Config config)
    {
        CANTalon4915 talon = new CANTalon4915(id, config);
        return talon;
    }

    /**
     * Run this on a fresh talon to produce good values for the defaults.
     */
    public static String getFullTalonInfo(CANTalon4915 talon)
    {
          return talon.dumpState();
    }
}
