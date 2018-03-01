package com.spartronics4915.lib.util.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Driver for a set of Sharp analog IR sensors that provide distance
 * measurement over a set range.
 * 
 * Usage: SpartIRSensor(int port)
 * 
 * where port = analog input port number on RoboRio
 * 
 * minVoltage = minimum voltage for range
 * 
 * maxVoltage = maximum voltage for range
 * 
 * Usage example:
 *
 * import com.spartronics4915.lib.util.drivers.SpartIRSensor;
 *
 * public class Harvester extends Subsystem
 * {
 *
 * private SpartIRSensor mSensor = new SpartIRSensor(kHavesterIRId);
 *
 * (in onLoop):
 *
 * double voltage = mSensor.getVoltage();
 * boolean acquired = false;
 * dashboardPutNumber("IRVoltage", voltage);
 * if (mSensor.isTargetInVoltageRange(voltage, kMinVoltage, kMaxVoltage)) {
 * acquired = true;
 * } else {
 * acquired = false;
 * }
 * dashboardPutBoolean("Acquired", acquired);
 * 
 * - or -
 * double inches = mSensor.getDistance();
 * boolean acquired = false;
 * dashboardPutNumber("IRDistance", inches);
 * if (mSensor.isTargetInDistanceRange(voltage, kMinDistance, kMaxDistance)) {
 * acquired = true;
 * } else {
 * acquired = false;
 * }
 * dashboardPutBoolean("Acquired", acquired);
 */
public class SpartIRSensor
{

    AnalogInput mAnalogInput;
    final double kTriggerVoltage = 1.1;

    public SpartIRSensor(int port)
    {
        mAnalogInput = new AnalogInput(port);
        mAnalogInput.setAverageBits(6);
    }

    public double getVoltage()
    {
        double v = mAnalogInput.getAverageVoltage();
        if (v < .001)
            v = .001;
        return v;
    }

    public double getDistance() // inches
    {
        // formula for sharp a41 detector, each model has a different formula
        //      v = 1 / (L + .42)  (1/cm)
        // 
        //double cm = 1.0 / getVoltage() - .42; // warning blows up when v == 0
        double volt = getVoltage();
        double cm = 59.06 - (94.11 * volt) + (57.60 * Math.pow(volt, 2.0)) - (11.65 * Math.pow(volt, 3.0));
        return cm / 2.54;
    }

    public boolean isTargetInVoltageRange(double min, double max)
    {
        double v = getVoltage();
        if (v > min && v < max)
            return true;
        else
            return false;
    }

    /**
     * @param minDist in inches
     * @param maxDist in inches
     * @return is within the distance
     */
    public boolean isTargetInDistanceRange(double minDist, double maxDist)
    {
        double d = getDistance();
        if (d > minDist && d < maxDist)
            return true;
        else
            return false;
    }
    
    public boolean isTargetAcquired()
    {
        double voltage = getVoltage();
        if (voltage > kTriggerVoltage)
            return true;
        else
            return false;
    }
}
