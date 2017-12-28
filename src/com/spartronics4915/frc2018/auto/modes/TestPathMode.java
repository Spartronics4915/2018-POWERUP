package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.DrivePathAction;
import com.spartronics4915.frc2018.auto.actions.ResetPoseFromPathAction;
import com.spartronics4915.frc2018.auto.actions.WaitAction;
import com.spartronics4915.frc2018.paths.PathContainer;
import com.spartronics4915.frc2018.paths.TestPath;

public class TestPathMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer testPath = new TestPath();
        runAction(new ResetPoseFromPathAction(testPath));
        runAction(new WaitAction(2)); // Give everything time to get reset
        runAction(new DrivePathAction(testPath));
    }

}
