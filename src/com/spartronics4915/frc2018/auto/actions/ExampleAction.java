package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Example;

public class ExampleAction implements Action
{
    Example mExample = Example.getInstance();
    
    @Override
    public boolean isFinished()
    {
        // Most actions should end, this is for demonstrations purposes only
        return false;
    }

    @Override
    public void update()
    {
        // Nothing happens here because we just need to set on or off once
    }

    @Override
    public void done()
    {
        // This will never get called in this case because isFinished never
        //  returns true, but it gets called after isFinished returns true.
        mExample.setOff();
    }

    @Override
    public void start()
    {
        mExample.setOn();
    }

}
