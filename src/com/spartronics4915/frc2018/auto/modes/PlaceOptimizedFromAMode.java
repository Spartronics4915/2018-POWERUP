package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ActuateArticulatedGrabberAction;
import com.spartronics4915.frc2018.auto.actions.ActuateHarvesterAction;
import com.spartronics4915.frc2018.auto.actions.ActuateScissorLiftAction;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ForceEndPathAction;
import com.spartronics4915.frc2018.auto.actions.ParallelAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.SeriesAction;
import com.spartronics4915.frc2018.auto.actions.TransferCubeFromGroundAction;
import com.spartronics4915.frc2018.auto.actions.TurnToHeadingAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.auto.actions.WaitForPathMarkerAction;
import com.spartronics4915.frc2018.paths.DriveSecondCubeToCSwitchPath;
import com.spartronics4915.frc2018.paths.DriveToCloseScaleFromAPath;
import com.spartronics4915.frc2018.paths.DriveToCloseSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveToFarSwitchFromAPath;
import com.spartronics4915.frc2018.paths.DriveSecondCubeToSwitchFromAScalePath;
import com.spartronics4915.frc2018.paths.DriveReverseToSecondCubeFromCSwitchPath;
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
        if (Util.getGameSpecificMessage().charAt(1) == 'L')
        {
            path = mCloseScalePath;
            liftPosition = ScissorLift.WantedState.SCALE;
            timeout = PowerupHelper.kCloseScaleTimeout;
            grabberPosition = ArticulatedGrabber.WantedState.TRANSPORT;
            doesTurn = true;
        }
        else if (Util.getGameSpecificMessage().charAt(0) == 'L')
        {
            path = mCloseSwitchPath;
            liftPosition = ScissorLift.WantedState.OFF;
            timeout = PowerupHelper.kSideSwitchCloseTimeout;
        }
        else
        {
            path = mFarSwitchPath;
            liftPosition = ScissorLift.WantedState.OFF;
            timeout = PowerupHelper.kSideSwitchFarTimeout;
        }
        runAction(new ResetPoseFromPathAction(path));
        runAction(PowerupHelper.getDriveAndArticulateActionWithTimeout(path, timeout, grabberPosition));
        runAction(new ActuateScissorLiftAction(liftPosition));
        if (doesTurn)
            runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(-90)));
        runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
        if (Util.getGameSpecificMessage().charAt(1) == 'L')
        {
            PathContainer secondPath = new DriveSecondCubeToSwitchFromAScalePath();
            runAction(new WaitAction(0.7));
            runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
            runAction(new ActuateScissorLiftAction(ScissorLift.WantedState.OFF));
            runAction(new ParallelAction(new DrivePathAction(secondPath),
                    new SeriesAction(new WaitForPathMarkerAction("acquirecube"), new ForceEndPathAction())));
            runAction(new TransferCubeFromGroundAction());
            runAction(new DrivePathAction(Util.truncatePathContainerUntilMarker(secondPath, "acquirecube")));
            if (Util.getGameSpecificMessage().charAt(0) == 'L')
                runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
        }
        else if (Util.getGameSpecificMessage().charAt(0) == 'N') // TODO: Add a way to pick up a second cube if we went to the scale
        {
            PathContainer secondPath = new DriveSecondCubeToCSwitchPath();
            runAction(new ParallelAction(new SeriesAction(new WaitForPathMarkerAction("openharvester"), new ActuateHarvesterAction(Harvester.WantedState.OPEN)),
                    new DrivePathAction(new DriveReverseToSecondCubeFromCSwitchPath())));
            runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
            runAction(new ParallelAction(new SeriesAction(new WaitForPathMarkerAction("acquirecube"), new ForceEndPathAction()),
                    new DrivePathAction(secondPath)));
            runAction(new ActuateHarvesterAction(Harvester.WantedState.HARVEST));
            runAction(new TransferCubeFromGroundAction());
            runAction(new DrivePathAction(Util.truncatePathContainerUntilMarker(secondPath, "acquirecube")));
            runAction(new ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState.RELEASE_CUBE));
        }
    }

}
