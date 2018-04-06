package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ActuateHarvesterAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromBPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromBPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.lib.util.Util;

public class PlaceSwitchFromBMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseSwitchFromBPath();
    private PathContainer mFarPath = new DriveToFarSwitchFromBPath();
    
    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(new ActuateHarvesterAction(Harvester.WantedState.GRAB));
        PathContainer path;
        if (Util.getGameSpecificMessage().charAt(0) == 'R')
        {
            path = mFarPath;
        }
        else
        {
            path = mClosePath;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(PowerupHelper.getDriveSwitchActionWithTimeout(path, PowerupHelper.kMiddleSwitchTimeout));
        runAction(new ActuateHarvesterAction(Harvester.WantedState.SLIDE_DROP));
        runAction(new ActuateHarvesterAction(Harvester.WantedState.OPEN));
    }

}
