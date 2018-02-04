package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.TurnToHeadingAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.DriveToFarScalePath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.lib.util.math.Rotation2d;

public class DriveToScaleMode extends AutoModeBase
{
    @Override
    protected void routine() throws AutoModeEndedException
    {
        // TODO: Switch paths based on field data
        PathContainer path = new DriveToFarScalePath();
        runAction(new ResetPoseFromPathAction(path));
        runAction(new WaitAction(2)); // Give everything time to get reset
        runAction(new DrivePathAction(path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
    }
    
}
