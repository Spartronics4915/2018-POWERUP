package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.DriveOpenLoopDurationAction;
import com.spartronics4915.frc2018.auto.actions.PrintDebugAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.lib.util.DriveSignal;

public class StressMotorsMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        for (double i = 0; i <= 10; i++)
        {
            runAction(new PrintDebugAction("Stress run index " + i + ": FORWARD"));
            runAction(new DriveOpenLoopDurationAction(new DriveSignal(1, 1), 120));
            runAction(new WaitAction(60));
            runAction(new PrintDebugAction("Stress run index " + i + ": REVERSE"));
            runAction(new DriveOpenLoopDurationAction(new DriveSignal(-1, -1), 120));
            runAction(new WaitAction(60));
            runAction(new PrintDebugAction("Stress run index " + i + " finished."));
        }
    }

}
