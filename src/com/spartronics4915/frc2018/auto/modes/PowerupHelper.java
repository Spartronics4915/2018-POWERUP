package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.actions.Action;
import com.spartronics4915.frc2018.auto.actions.ActuateHarvesterAction;
import com.spartronics4915.frc2018.auto.actions.ActuateScissorLiftAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ParallelAction;
import com.spartronics4915.frc2018.auto.actions.ParallelSingleWaitAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.frc2018.subsystems.ScissorLift;

public class PowerupHelper
{
    public static final double kSideSwitchFarTimeout = 13;
    public static final double kSideSwitchCloseTimeout = 5;
    public static final double kMiddleSwitchTimeout = 4;
    
    public static final double kCloseScaleTimeout = 13;
    
    public static Action getDriveSwitchActionWithTimeout(PathContainer path, double timeout)
    {
        return new ParallelAction(
                new ActuateScissorLiftAction(ScissorLift.WantedState.SWITCH),
                new ParallelSingleWaitAction(new WaitAction(timeout), new DrivePathAction(path)));
    }
}
