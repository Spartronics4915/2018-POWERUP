package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.TurnToHeadingAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.lib.util.math.Rotation2d;

public class TestTurnMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(2));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90)));
    }
}
