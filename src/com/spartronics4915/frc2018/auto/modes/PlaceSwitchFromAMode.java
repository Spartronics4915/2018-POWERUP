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
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromAPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.lib.util.Util;

public class PlaceSwitchFromAMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseSwitchFromAPath();
    private PathContainer mFarPath = new DriveToFarSwitchFromAPath();

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        double timeout;
        if (Util.getGameSpecificMessage().charAt(0) == 'L')
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
