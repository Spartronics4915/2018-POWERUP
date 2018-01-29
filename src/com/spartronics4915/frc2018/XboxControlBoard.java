package com.spartronics4915.frc2018;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControlBoard implements ControlBoardInterface
{

    private final XboxController mController;

    public XboxControlBoard()
    {
        mController = new XboxController(0);
    }

    @Override
    public double getThrottle()
    {
        return mController.getY(GenericHID.Hand.kLeft);
    }

    @Override
    public double getTurn()
    {
        return mController.getX(GenericHID.Hand.kRight);
    }

    @Override
    public boolean getQuickTurn()
    {
        return mController.getAButton();
    }

    @Override
    public boolean getLowGear()
    {
        return mController.getBButton();
    }
}
