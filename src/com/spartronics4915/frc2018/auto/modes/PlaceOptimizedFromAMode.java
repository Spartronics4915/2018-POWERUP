package com.spartronics4915.frc2018.auto.modes;

import java.util.ArrayList;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.Action;
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
import com.spartronics4915.frc2018.paths.DriveSecondCubeToAScalePath;
import com.spartronics4915.frc2018.paths.DriveToCloseScaleFromAPath;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarScaleFromAPath;
import com.spartronics4915.frc2018.paths.DriveToSecondCubeFromASwitchPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.lib.util.Util;

public class PlaceOptimizedFromAMode extends AutoModeBase
{
    private PathContainer mCloseScalePath = new DriveToCloseScaleFromAPath();
    private PathContainer mCloseSwitchPath = new DriveToCloseSwitchFromAPath();
    private PathContainer mFarScalePath = new DriveToFarScaleFromAPath();

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        if (Util.getGameSpecificMessage().charAt(0) == 'L')
        {
            path = mCloseSwitchPath;
        }
        else if (Util.getGameSpecificMessage().charAt(1) == 'L')
        {
            path = mCloseScalePath;
        }
        else
        {
            path = mFarScalePath;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(new WaitAction(0.1)); // Give everything time to get reset
        runAction(new DrivePathAction(path));
        runAction(new ActuateScissorLiftAction(ScissorLift.WantedState.SCALE));
        runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
        if (Util.getGameSpecificMessage().charAt(0) == 'L') // TODO: Add a way to pick up a second cube if we went to the scale
        {
            runAction(new ParallelAction(new SeriesAction(new WaitForPathMarkerAction("openharvester"), new ActuateHarvesterAction(Harvester.WantedState.OPEN)),
                    new DrivePathAction(new DriveToSecondCubeFromASwitchPath())));
            runAction(new ActuateHarvesterAction(Harvester.WantedState.HARVEST));
            runAction(new TransferCubeFromGroundAction());
        }
        else
        {
            return;
        }
        
        PathContainer secondPath;
        if (Util.getGameSpecificMessage().charAt(1) == 'L')
        {
            secondPath = new DriveSecondCubeToAScalePath();
        }
        else if (Util.getGameSpecificMessage().charAt(0) == 'L')
        {
            secondPath = new DriveSecondCubeToAScalePath();
        }
        else
        {
            return;
        }
        runAction(new DrivePathAction(secondPath));
    }

}
