package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.DriveSys;
import com.spartronics4915.lib.util.DriveSignal;

public class DriveOpenLoopAction implements Action
{
    DriveSys mDrive = DriveSys.getInstance();
    
    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void update()
    {
    }

    @Override
    public void done()
    {
    }

    @Override
    public void start()
    {
        mDrive.setOpenLoop(new DriveSignal(0.5, 0.5));
    }
    
}
