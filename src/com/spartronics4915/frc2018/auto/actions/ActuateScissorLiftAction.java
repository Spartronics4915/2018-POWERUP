package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.lib.util.Logger;

public class ActuateScissorLiftAction implements Action
{

    ScissorLift mLifter = null;
    ScissorLift.WantedState mWantedState;
    
    public ActuateScissorLiftAction(ScissorLift.WantedState wantedState)
    {
        mWantedState = wantedState;
    }
    
    @Override
    public boolean isFinished()
    {
        return mLifter.atTarget();
    }

    @Override
    public void update()
    {
        // do nothing
    }

    @Override
    public void done()
    {
        // We don't want to change the scissor position at the end.
        // This should be done explicitly by the user instead.
        Logger.notice("Done actuating scissor lift.");
    }

    @Override
    public void start()
    {
        mLifter = ScissorLift.getInstance();
        mLifter.setWantedState(mWantedState);
        Logger.notice("Scissor lift actuating to "+mWantedState.toString()+".");
    }

}
