package com.spartronics4915.frc2018;

import edu.wpi.first.wpilibj.Joystick;

public class AtlasControlBoard implements ControlBoardInterface
{
    // TODO: Buttons still seem broken
    
    private Joystick mFlightstick = new Joystick(0);

    @Override
    public double getThrottle()
    {
        return mFlightstick.getY();
    }

    @Override
    public double getTurn()
    {
        return mFlightstick.getX();
    }

    @Override
    public boolean getQuickTurn()
    {
        return mFlightstick.getRawButtonPressed(2);
    }

    @Override
    public boolean getLowGear()
    {
        return mFlightstick.getTriggerPressed();
    }
}
