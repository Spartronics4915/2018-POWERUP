package com.spartronics4915.lib.util.drivers;

import edu.wpi.first.wpilibj.MotorSafety;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Creates CANTalon objects and configures all the parameters we care about to
 * factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class CANTalonFactory
{

    public static class Configuration
    {

        public boolean LIMIT_SWITCH_NORMALLY_OPEN = true;
        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0;
        public double PEAK_VOLTAGE = 12;
        public boolean ENABLE_BRAKE = false;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public int FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public double NOMINAL_CLOSED_LOOP_VOLTAGE = 12;
        public int REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double VOLTAGE_COMPENSATION_RAMP_RATE = 0;
        public double VOLTAGE_RAMP_RATE = 0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static
    {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static CANTalon4915 createDefaultTalon(int id)
    {
        return createTalon(id, kDefaultConfiguration);
    }

    public static CANTalon4915 createPermanentSlaveTalon(int id, int master_id)
    {
        final CANTalon4915 talon = createTalon(id, kSlaveConfiguration);
        talon.changeControlMode(ControlMode.Follower);
        talon.set(master_id);
        return talon;
    }

    public static CANTalon4915 createTalon(int id, Configuration config)
    {
        CANTalon4915 talon = new CANTalon4915(id);
        talon.changeControlMode(ControlMode.PercentOutput);
        // XXX: apply configuration
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
