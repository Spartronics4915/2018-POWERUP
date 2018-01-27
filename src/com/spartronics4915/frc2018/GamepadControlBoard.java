package com.spartronics4915.frc2018;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Contains the button mappings for the Gamepad control board. Like the drive
 * code, one instance of the GamepadControlBoard
 * object is created upon startup, then other methods request the singleton
 * GamepadControlBoard instance. Implements the
 * ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class GamepadControlBoard implements ControlBoardInterface
{

    private final XboxController mController;

    protected GamepadControlBoard()
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

    @Override
    public boolean getBlinkLEDButton()
    {
        return mController.getXButton();
    }

    @Override
    public boolean getIntakeButton()
    {
        return mController.getYButton();
    }
}
