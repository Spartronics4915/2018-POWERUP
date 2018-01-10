package com.spartronics4915.frc2018;

import edu.wpi.first.wpilibj.Joystick;

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

    private final Joystick mGamepad;

    protected GamepadControlBoard()
    {
        mGamepad = new Joystick(0);
    }

    @Override
    public double getThrottle()
    {
        return mGamepad.getRawAxis(1);
    }

    @Override
    public double getTurn()
    {
        return mGamepad.getRawAxis(4);
    }

    @Override
    public boolean getQuickTurn()
    {
        // R1
        return mGamepad.getRawButton(6);
    }

    @Override
    public boolean getLowGear()
    {
        // L1
        return mGamepad.getRawButton(5);

    }

    @Override
    public boolean getBlinkLEDButton()
    {
        return false;
    }
}
