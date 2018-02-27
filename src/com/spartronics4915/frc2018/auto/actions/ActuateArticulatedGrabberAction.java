package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.lib.util.Logger;

public class ActuateArticulatedGrabberAction implements Action
{

    ArticulatedGrabber mLifter = null;
    ArticulatedGrabber.WantedState mWantedState;
    
    public ActuateArticulatedGrabberAction(ArticulatedGrabber.WantedState wantedState)
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
        // We don't want to change the grabber position at the end.
        // This should be done explicitly by the user instead.
        Logger.notice("Done actuating articulated grabber.");
    }

    @Override
    public void start()
    {
        Logger.notice("Actuating articulated grabber to "+mWantedState.toString()+".");
        mLifter = ArticulatedGrabber.getInstance();
        mLifter.setWantedState(mWantedState);
    }

}
