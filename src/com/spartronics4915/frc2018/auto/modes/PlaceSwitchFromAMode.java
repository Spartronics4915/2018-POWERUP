package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromAPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.lib.util.Util;

public class PlaceSwitchFromAMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseSwitchFromAPath();
    private PathContainer mFarPath = new DriveToFarSwitchFromAPath();

    public PlaceSwitchFromAMode()
    {
        mClosePath.buildPath();
        mFarPath.buildPath();
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        if (Util.getGameSpecificMessage().charAt(0) == 'R')
        {
            path = mClosePath;
        }
        else
        {
            path = mFarPath;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(new WaitAction(0.1)); // Give everything time to get reset
        runAction(new DrivePathAction(path));
    }

}
