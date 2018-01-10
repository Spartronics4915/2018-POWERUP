package com.spartronics4915.lib.util.drivers;

import com.spartronics4915.lib.util.math.Rotation2d;

/**
 * This class implements some of the functionality missing from the BNO055
 * that is present in the NavX (mostly related to resetting and zeroing).
 * This class closely follows {@link NavX}, especially in method names,
 * and access and concurrency keywords.
 * 
 * @author declan
 * @see NavX
 * @see com.spartronics4915.lib.util.math.Rotation2d
 */
public class BetterBNO
{

    private BNO055 mBNO055;

    private Rotation2d mAngleZeroingOffset = Rotation2d.identity();
    private Rotation2d mAngleAdjustment = Rotation2d.identity(); // For the user to adjust the angle

    public BetterBNO()
    {
        mBNO055 = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);
    }

    public synchronized void reset()
    {
        mBNO055.resetState();
        mAngleZeroingOffset = Rotation2d.identity(); // Set the offset to zero
    }

    public synchronized void zeroYaw()
    {
        // We use the offset to zero ourselves
        mAngleZeroingOffset = Rotation2d.fromDegrees(mBNO055.getNormalizedHeading()).inverse();
    }

    public synchronized void setAngleAdjustment(Rotation2d adjustment)
    {
        mAngleAdjustment = adjustment;
    }

    public synchronized double getRawYawDegrees()
    {
        return mBNO055.getNormalizedHeading();
    }

    public Rotation2d getYaw()
    {
        return mAngleZeroingOffset.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees())).rotateBy(mAngleAdjustment);
    }

    public double getRawAccelX()
    {
        return mBNO055.getAccel()[0]; // XXX: I'm pretty sure that index 0 is the X-axis, but there isn't any documentation on it
    }

    public boolean isPresent()
    {
        return mBNO055.isSensorPresent();
    }
}
