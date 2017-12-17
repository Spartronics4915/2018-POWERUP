package com.team254.frc2017.auto.actions;

import java.util.Optional;

import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.subsystems.LED;
import com.team254.lib.util.math.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Turns the robot towards a target heading until the robot either reaches that heading or sees the boiler.
 *
 * @see Action
 */
public class TurnUntilSeesTargetAction extends TurnToHeadingAction {

    RobotState mState = RobotState.getInstance();

    public TurnUntilSeesTargetAction(Rotation2d heading) {
        super(heading);
        LED.getInstance().setWantedState(LED.WantedState.FIND_RANGE);
        // TODO Auto-generated constructor stub
    }

    @Override
    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();
        Optional<ShooterAimingParameters> aimParams = mState.getAimingParameters();
        if (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5) {
            return true;
        }
        return super.isFinished();
    }

}
