package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.*;
import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;

public class TestDrivePIDMode extends AutoModeBase
{
    private String mVariant;

    public TestDrivePIDMode(String variant)
    {
        this.mVariant = variant;
    }
    
    @Override
    protected void routine() throws AutoModeEndedException
    {
        if(mVariant.equals("velocity"))
        {
            // drive signal measured in inches/sec
            Logger.notice("TestDrivePIDMode velocity begin");
          
            // fwd 8 ips, 4 sec
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(8,8), 4));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));
            
            // rev 8 ips, 4 sec
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(-8,-8), 4));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));
          
            // rotate right, then left 4 sec
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(4,-4), 4));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(-4,4), 4));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));
            
            // moderate left curve, 4 sec
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(3,4), 4));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));

            // moderate right curve, 4 sec
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(4,3), 4));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));
            
            // mixed velocity
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(4,4), 2));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(8,8), 2));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(1,1), 2));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(-4,-4), 2));
            
           Logger.notice("TestDrivePIDMode velocity end");
        }
        else
        if(mVariant.equals("position"))
        {
            // drive signal measured in inches
            // nb: in position mode, we express a max velocity and max accel. This
            //  is done within the confines of Drive (and Constants.java)
            Logger.notice("TestDrivePIDMode position begin");
            
            // fwd, then rev 36 inches (wheels are 6 inch diameter)
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(36,36), 6));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(-36,-36), 6));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));
           
            // right, then left rotate.
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(36,-36), 6));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(-36, 36), 6));
            runAction(new DriveClosedLoopAction(mVariant, new DriveSignal(0, 0), 2));
            Logger.notice("TestDrivePIDMode position end");            
        }
        else
        {
            throw new AutoModeEndedException();
        }
        Drive.getInstance().setOpenLoop(new DriveSignal(0, 0, true));
    }
}
