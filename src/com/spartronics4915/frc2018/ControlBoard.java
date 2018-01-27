package com.spartronics4915.frc2018;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Contains the button mappings for the competition control board. Like the
 * drive code, one instance of the ControlBoard
 * object is created upon startup, then other methods request the singleton
 * ControlBoard instance. Implements the
 * ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class ControlBoard implements ControlBoardInterface
{

    private static ControlBoardInterface mInstance = null;

    private static final boolean kUseGamepad = true;

    public static ControlBoardInterface getInstance()
    {
        if (mInstance == null)
        {
            if (kUseGamepad)
            {
                mInstance = new GamepadControlBoard();
            }
            else
            {
                mInstance = new ControlBoard();
            }
        }
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;
    private final Joystick mButtonBoard;

    protected ControlBoard()
    {
        mThrottleStick = new Joystick(0);
        mTurnStick = new Joystick(1);
        mButtonBoard = new Joystick(2);
    }

    // DRIVER CONTROLS
    @Override
    public double getThrottle()
    {
        return -mThrottleStick.getRawAxis(0);
    }

    @Override
    public double getTurn()
    {
        return -mTurnStick.getY();
    }

    @Override
    public boolean getQuickTurn()
    {
        return mTurnStick.getRawButton(1);
    }

    @Override
    public boolean getLowGear()
    {
        return mThrottleStick.getRawButton(2);
    }

    @Override
    public boolean getBlinkLEDButton()
    {
        return mButtonBoard.getRawButton(9);
    }

    @Override
    public boolean getIntakeButton()
    {
        // See XboxControlBoard
        return false;
    }
}
