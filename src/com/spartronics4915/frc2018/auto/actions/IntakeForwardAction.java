package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Testbed;
import com.spartronics4915.frc2018.subsystems.Testbed.WantedState;

public class IntakeForwardAction implements Action
{
    Testbed mTestBed = Testbed.getInstance();
    
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
        mTestBed.setWantedState(WantedState.FORWARD_INTAKE);
    }

}
