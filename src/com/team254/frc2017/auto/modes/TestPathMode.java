package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.TestPath;

public class TestPathMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(2));
        PathContainer testPath = new TestPath();
        runAction(new ResetPoseFromPathAction(testPath));
        runAction(new DrivePathAction(testPath));
    }

}
