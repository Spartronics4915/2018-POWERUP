package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ActuateHarvesterAction;
import com.spartronics4915.frc2018.auto.actions.ActuateScissorLiftAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ForceEndPathAction;
import com.spartronics4915.frc2018.auto.actions.ParallelAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.SeriesAction;
import com.spartronics4915.frc2018.auto.actions.TurnToHeadingAction;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.auto.actions.WaitForPathMarkerAction;
import com.spartronics4915.frc2018.paths.DriveSecondCubeToSwitchFromCScalePath;
import com.spartronics4915.frc2018.paths.DriveToCloseScaleFromCPath;
import com.spartronics4915.frc2018.paths.DriveToFarScaleFromCPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.math.Rotation2d;

public class PlaceScaleFromCMode extends AutoModeBase
{

    private PathContainer mClosePath = new DriveToCloseScaleFromCPath();
    private PathContainer mFarPath = new DriveToFarScaleFromCPath();

    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(new ActuateHarvesterAction(Harvester.WantedState.GRAB));
        PathContainer path;
        if (Util.getGameSpecificMessage().charAt(1) == 'R')
        {
            path = mClosePath;
        }
        else
        {
            path = mFarPath;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(new ParallelAction(new DrivePathAction(path), 
                new ActuateScissorLiftAction(ScissorLift.WantedState.SCALE),
                new ActuateHarvesterAction(Harvester.WantedState.SLIDE_DROP)));
        if (Util.getGameSpecificMessage().charAt(1) == 'R')
        {
            runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90)));
        }
        runAction(new ActuateHarvesterAction(Harvester.WantedState.OPEN));
        if (Util.getGameSpecificMessage().charAt(1) == 'R')
        {
            PathContainer secondPath = new DriveSecondCubeToSwitchFromCScalePath();
            runAction(new WaitAction(0.3));
            runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
            runAction(new ActuateHarvesterAction(Harvester.WantedState.GRAB));
            runAction(new ActuateScissorLiftAction(ScissorLift.WantedState.OFF));
            runAction(new ActuateHarvesterAction(Harvester.WantedState.DEPLOY));
            runAction(new ParallelAction(new DrivePathAction(secondPath),
                    new SeriesAction(new WaitForPathMarkerAction("harvest"), new ActuateHarvesterAction(Harvester.WantedState.OPEN)),
                    new SeriesAction(new WaitForPathMarkerAction("acquirecube"), new ForceEndPathAction())));
            runAction(new ActuateHarvesterAction(Harvester.WantedState.GRAB));
            if (Util.getGameSpecificMessage().charAt(0) == 'R')
                runAction(new ActuateScissorLiftAction(ScissorLift.WantedState.SWITCH));
                runAction(new DrivePathAction(Util.truncatePathContainerUntilMarker(secondPath, "acquirecube")));
                runAction(new ActuateHarvesterAction(Harvester.WantedState.EJECT));
        }
    }

}
