package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.actions.Action;
import com.spartronics4915.frc2018.auto.actions.ActuateArticulatedGrabberAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ParallelAction;
import com.spartronics4915.frc2018.auto.actions.ParallelSingleWaitAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;

public class PowerupHelper
{
    public static final double kSideSwitchFarTimeout = 10;
    public static final double kSideSwitchCloseTimeout = 13;
    
    public static Action getDriveSwitchActionWithTimeout(PathContainer path, double timeout)
    {
        return new ParallelAction(
                new ParallelSingleWaitAction(new WaitAction(timeout), new DrivePathAction(path)),
                new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.PREPARE_DROP));
    }
}
