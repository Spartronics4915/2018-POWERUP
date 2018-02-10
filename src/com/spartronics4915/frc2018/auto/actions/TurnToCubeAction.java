package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Drive;

public class TurnToCubeAction implements Action
{

    Drive mDrive;
    
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
        mDrive.stop();
    }

    @Override
    public void start()
    {
        mDrive = Drive.getInstance();
        mDrive.setWantAimToVisionTarget();
    }

}
