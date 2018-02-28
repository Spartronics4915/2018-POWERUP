package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ActuateArticulatedGrabberAction;
import com.spartronics4915.frc2018.auto.actions.ActuateHarvesterAction;
import com.spartronics4915.frc2018.auto.actions.ActuateScissorLiftAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ParallelAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.SeriesAction;
import com.spartronics4915.frc2018.auto.actions.TransferCubeFromGroundAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.auto.actions.WaitForPathMarkerAction;
import com.spartronics4915.frc2018.paths.DriveSecondCubeToCScalePath;
import com.spartronics4915.frc2018.paths.DriveSecondCubeToCSwitchPath;
import com.spartronics4915.frc2018.paths.DriveToCloseScaleFromCPath;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromCPath;
import com.spartronics4915.frc2018.paths.DriveToFarScaleFromCPath;
import com.spartronics4915.frc2018.paths.DriveReverseToSecondCubeFromCSwitchPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.lib.util.Util;

public class PlaceOptimizedFromCMode extends AutoModeBase
{
    private PathContainer mCloseScalePath = new DriveToCloseScaleFromCPath();
    private PathContainer mCloseSwitchPath = new DriveToCloseSwitchFromCPath();
    private PathContainer mFarScalePath = new DriveToFarScaleFromCPath();

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        if (Util.getGameSpecificMessage().charAt(0) == 'R')
        {
            path = mCloseSwitchPath;
        }
        else if (Util.getGameSpecificMessage().charAt(1) == 'R')
        {
            path = mCloseScalePath;
        }
        else
        {
            path = mFarScalePath;
        }
        // BUG BUG BUG! All of below is pretty wrong. Paths should really be renamed, etc.
        runAction(new ResetPoseFromPathAction(path));
        runAction(new DrivePathAction(path));
        runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
        if (Util.getGameSpecificMessage().charAt(0) == 'R') // TODO: Add a way to pick up a second cube if we went to the scale
        {
            runAction(new ParallelAction(new SeriesAction(new WaitForPathMarkerAction("openharvester"), new ActuateHarvesterAction(Harvester.WantedState.OPEN)),
                    new DrivePathAction(new DriveReverseToSecondCubeFromCSwitchPath())));
            runAction(new ActuateHarvesterAction(Harvester.WantedState.HARVEST));
            runAction(new TransferCubeFromGroundAction());
        }
        else
        {
            return;
        }
        
        PathContainer secondPath;
        if (Util.getGameSpecificMessage().charAt(1) == 'R')
        {
            secondPath = new DriveSecondCubeToCScalePath();
        }
        else if (Util.getGameSpecificMessage().charAt(0) == 'R')
        {
            secondPath = new DriveSecondCubeToCSwitchPath();
        }
        else
        {
            return;
        }
        runAction(new DrivePathAction(secondPath));
    }

}
