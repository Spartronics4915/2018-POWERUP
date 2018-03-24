package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Harvester;

public class ActuateHarvesterAction implements Action
{

    Harvester mHarvester = null;
    Harvester.WantedState mWantedState;
    
    public ActuateHarvesterAction(Harvester.WantedState wantedState)
    {
        mWantedState = wantedState;
    }
    
    @Override
    public boolean isFinished()
    {
        return mHarvester.atTarget();
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
    }

    @Override
    public void start()
    {
        mHarvester = Harvester.getInstance();
        mHarvester.setWantedState(mWantedState);
    }

}
