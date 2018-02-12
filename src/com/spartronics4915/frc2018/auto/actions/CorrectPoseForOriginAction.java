package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Convert field coordinates as if you've started from a different origin.
 * For example, if my auto mode always starts at blue 3, but some of them
 * work where you can place the robot at red 3, this will correct your
 * coordinates (after you've run the path) so they actually represent
 * where the robot is on the field.
 * 
 * @author declan
 */
public class CorrectPoseForOriginAction extends RunOnceAction
{

    public enum FieldLandmarks {
        A, B, C,
    }
    
    public enum FieldSides {
        BLUE, RED,
    }
    
    public class FieldPosition {
        public FieldLandmarks landmark;
        public FieldSides side;
    }
    
    private FieldPosition mFrom;
    private FieldPosition mTo;
    
    public CorrectPoseForOriginAction(FieldPosition from, FieldPosition to) {
        mFrom = from;
        mTo = to;
    }

    @Override
    public void runOnce()
    {
        RigidTransform2d robotTransform = RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp());
        if (mFrom.side != mTo.side) {
            robotTransform.getTranslation().setX(Constants.kFieldWidth - robotTransform.getTranslation().x());
//            if (mFrom.side == FieldSides.BLUE) TODO
//                robotTransform.getRotation().rotateBy(Rotation2d.fromRadians(-Math.PI/2));
//            else
//                robotTransform.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI/2));
        }
        if ((mFrom.landmark == FieldLandmarks.A && mTo.landmark == FieldLandmarks.C) || (mFrom.landmark == FieldLandmarks.C && mTo.landmark == FieldLandmarks.C)) {
            robotTransform.getTranslation().setY(Constants.kFieldHeight - robotTransform.getTranslation().y());
//            if (mFrom.landmark == FieldLandmarks.C)
//                robotTransform.getRotation().rotateBy(Rotation2d.fromRadians(-Math.PI/2));
//            else
//                robotTransform.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI/2));
        }
    }

}
