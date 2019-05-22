package com.spartronics4915.lib.util;

public class ArcadeDriveHelper
{

    /**
     * This class holds a simple arcade drive helper method, where the driver
     * specifies speed along the X axis, and rotation rate around the Z axis.
     * (Where Z points up, and X points forward.)
     *
     * Code taken from wpilib and modified a little:
     * https://github.com/wpilibsuite/allwpilib/blob/88a09dd13ab01f2a6f3443f6488f4c3ac39531ef/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java#L172
     */

    private static final double kJoystickDeadband = 0.02;

    public static DriveSignal arcadeDrive(double xSpeed, double zRotation, boolean isSlowMode)
    {
        xSpeed = limit(xSpeed);
        xSpeed = applyDeadband(xSpeed);

        zRotation = limit(zRotation);
        zRotation = applyDeadband(zRotation);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (!isSlowMode)
        {
            xSpeed = Math.copySign(Math.pow(Math.abs(xSpeed), 5.0/3.0), xSpeed);
            zRotation = Math.copySign(Math.pow(Math.abs(zRotation), 5.0/2.0), zRotation);
        }
        else
        {
            xSpeed *= 0.5;
            zRotation *= 0.4;
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0)
        {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0)
            {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
            else
            {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        }
        else
        {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0)
            {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
            else
            {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        return new DriveSignal(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }

    /**
     * This is a convenience method to ensure that we only send the motors numbers
     * in a reasonable range.
     *
     * @param input is any number that will eventually find its way to the motors
     * @return input restricted on the set of reals [0, 1]
     */
    private static double limit(double input)
    {
        if (input > 1)
            return 1;
        else if (input < -1)
            return -1;
        return input;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * Lifted from wpilib.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private static double applyDeadband(double value)
    {
        if (Math.abs(value) > kJoystickDeadband)
        {
            if (value > 0.0)
                return (value - kJoystickDeadband) / (1.0 - kJoystickDeadband);
            else
                return (value + kJoystickDeadband) / (1.0 - kJoystickDeadband);
        }
        else
            return 0.0;
    }
}
