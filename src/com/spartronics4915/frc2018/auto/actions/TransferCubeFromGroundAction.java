package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Superstructure;
import com.spartronics4915.frc2018.subsystems.Superstructure.WantedState;

public class TransferCubeFromGroundAction implements Action
{

    Superstructure mSuperstructure;
    
    @Override
    public boolean isFinished()
    {
        return mSuperstructure.isIdling();
    }

    @Override
    public void update()
    {
        // nothing happens here
    }

    @Override
    public void done()
    {
    }

    @Override
    public void start()
    {
        mSuperstructure = Superstructure.getInstance();
        mSuperstructure.setWantedState(WantedState.TRANSFER_CUBE_TO_GRABBER);
    }

}
