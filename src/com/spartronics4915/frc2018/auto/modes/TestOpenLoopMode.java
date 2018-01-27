package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.DriveOpenLoopDurationAction;
import com.spartronics4915.lib.util.DriveSignal;

public class TestOpenLoopMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(new DriveOpenLoopDurationAction(new DriveSignal(0.5, 0.5), 60));
    }

}
