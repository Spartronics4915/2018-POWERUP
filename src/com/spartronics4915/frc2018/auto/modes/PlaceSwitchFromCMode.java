package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ActuateArticulatedGrabberAction;
import com.spartronics4915.frc2018.auto.actions.ActuateScissorLiftAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromCPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromCPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.lib.util.Util;

public class PlaceSwitchFromCMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseSwitchFromCPath();
    private PathContainer mFarPath = new DriveToFarSwitchFromCPath();

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
        runAction(new ActuateScissorLiftAction(ScissorLift.WantedState.SWITCH));
        runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
    }

}
