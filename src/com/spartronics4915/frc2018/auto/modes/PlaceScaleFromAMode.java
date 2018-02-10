package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.TurnToHeadingAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.DriveToCloseScaleFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarScaleFromAPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.lib.util.math.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;

public class PlaceScaleFromAMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseScaleFromAPath();
    private PathContainer mFarPath = new DriveToFarScaleFromAPath();

    public PlaceScaleFromAMode()
    {
        mClosePath.buildPath();
        mFarPath.buildPath();
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        Rotation2d endTurn;
        if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'L')
        {
            path = mClosePath;
            endTurn = Rotation2d.fromDegrees(90);
        }
        else
        {
            path = mFarPath;
            endTurn = Rotation2d.fromDegrees(-90);
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(new WaitAction(0.1)); // Give everything time to get reset
        runAction(new DrivePathAction(path));
        runAction(new TurnToHeadingAction(endTurn));
    }

}
