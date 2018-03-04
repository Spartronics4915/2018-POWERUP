package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ActuateArticulatedGrabberAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ParallelAction;
import com.spartronics4915.frc2018.auto.actions.ParallelSingleWaitAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.SeriesAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromCPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromCPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.lib.util.Util;

public class PlaceSwitchFromCMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseSwitchFromCPath();
    private PathContainer mFarPath = new DriveToFarSwitchFromCPath();

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        double timeout;
        if (Util.getGameSpecificMessage().charAt(0) == 'R')
        {
            path = mClosePath;
            timeout = PowerupHelper.kSideSwitchCloseTimeout;
        }
        else
        {
            path = mFarPath;
            timeout = PowerupHelper.kSideSwitchFarTimeout;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(PowerupHelper.getDriveSwitchActionWithTimeout(path, timeout));
        runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
    }

}
