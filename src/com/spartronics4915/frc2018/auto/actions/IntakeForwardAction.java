package com.spartronics4915.frc2018.auto.actions;

<<<<<<< HEAD:src/com/spartronics4915/frc2018/auto/actions/IntakeAction.java
import com.spartronics4915.frc2018.subsystems.Testbed;
=======
import com.spartronics4915.frc2018.subsystems.Intake;
import com.spartronics4915.frc2018.subsystems.Intake.WantedState;
>>>>>>> c626604df71e99149b4d27fd2b4b32adfd0a9178:src/com/spartronics4915/frc2018/auto/actions/IntakeForwardAction.java

public class IntakeForwardAction implements Action
{
<<<<<<< HEAD:src/com/spartronics4915/frc2018/auto/actions/IntakeAction.java
    Testbed mExample = Testbed.getInstance();
=======
    Intake mIntake = Intake.getInstance();
>>>>>>> c626604df71e99149b4d27fd2b4b32adfd0a9178:src/com/spartronics4915/frc2018/auto/actions/IntakeForwardAction.java
    
    @Override
    public boolean isFinished()
    {
        // Most actions should end, this is for demonstrations purposes only
        return false;
    }

    @Override
    public void update()
    {
        // If you need to do something frequently, that should happen here
    }

    @Override
    public void done()
    {
        // This will never get called in this case because isFinished never
        //  returns true, but it gets called after isFinished returns true.
    }

    @Override
    public void start()
    {
        mIntake.setWantedState(WantedState.FORWARD_INTAKE);
    }

}
