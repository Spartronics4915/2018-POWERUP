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
import com.spartronics4915.frc2018.auto.actions.TurnToHeadingAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.auto.actions.WaitForPathMarkerAction;
import com.spartronics4915.frc2018.paths.DriveSecondCubeToAScalePath;
import com.spartronics4915.frc2018.paths.DriveToCloseScaleFromAPath;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarScaleFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveReverseToSecondCubeFromASwitchPath;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.math.Rotation2d;

public class PlaceOptimizedFromAMode extends AutoModeBase
{
    private PathContainer mCloseScalePath = new DriveToCloseScaleFromAPath();
    private PathContainer mCloseSwitchPath = new DriveToCloseSwitchFromAPath();
    private PathContainer mFarSwitchPath = new DriveToFarSwitchFromAPath();

    @Override
    protected void routine() throws AutoModeEndedException
    {
        PathContainer path;
        ScissorLift.WantedState liftPosition;
        ArticulatedGrabber.WantedState grabberPosition = ArticulatedGrabber.WantedState.PREPARE_DROP;
        double timeout;
        boolean doesTurn = false;
        if (Util.getGameSpecificMessage().charAt(0) == 'L')
        {
            path = mCloseSwitchPath;
            liftPosition = ScissorLift.WantedState.OFF;
            timeout = PowerupHelper.kSideSwitchCloseTimeout;
        }
        else if (Util.getGameSpecificMessage().charAt(1) == 'L')
        {
            path = mCloseScalePath;
            liftPosition = ScissorLift.WantedState.SCALE;
            timeout = PowerupHelper.kCloseScaleTimeout;
            grabberPosition = ArticulatedGrabber.WantedState.TRANSPORT;
            doesTurn = true;
        }
        else
        {
            path = mFarSwitchPath;
            liftPosition = ScissorLift.WantedState.OFF;
            timeout = PowerupHelper.kSideSwitchFarTimeout;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(PowerupHelper.getDriveAndArticulateActionWithTimeout(path, timeout, grabberPosition));
        if (doesTurn)
            runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(-90)));
        runAction(new ActuateScissorLiftAction(liftPosition));
        runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
//        if (Util.getGameSpecificMessage().charAt(0) == 'L') // TODO: Add a way to pick up a second cube if we went to the scale
//        {
//            runAction(new ParallelAction(new SeriesAction(new WaitForPathMarkerAction("openharvester"), new ActuateHarvesterAction(Harvester.WantedState.OPEN)),
//                    new DrivePathAction(new DriveReverseToSecondCubeFromASwitchPath())));
//            runAction(new ActuateHarvesterAction(Harvester.WantedState.HARVEST));
//            runAction(new TransferCubeFromGroundAction());
//        }
//        else
//        {
//            return;
//        }
//        
//        PathContainer secondPath;
//        if (Util.getGameSpecificMessage().charAt(1) == 'L')
//        {
//            secondPath = new DriveSecondCubeToAScalePath();
//        }
//        else if (Util.getGameSpecificMessage().charAt(0) == 'L')
//        {
//            secondPath = new DriveSecondCubeToAScalePath();
//        }
//        else
//        {
//            return;
//        }
//        runAction(new DrivePathAction(secondPath));
    }

}
