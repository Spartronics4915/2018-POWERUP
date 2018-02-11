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

    public static ControlBoardInterface getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private final Joystick mDrivestick;
    private final Joystick mButtonBoard; // Currently unused

    protected ControlBoard()
    {
        mDrivestick = new Joystick(0);
        mButtonBoard = null; // FIXME
    }

    @Override
    public double getThrottle()
    {
        return mDrivestick.getY();
    }

    @Override
    public double getTurn()
    {
        return mDrivestick.getX();
    }

    @Override
    public boolean getQuickTurn()
    {
        return mDrivestick.getRawButtonPressed(2);
    }

    @Override
    public boolean getLowGear()
    {
        return mDrivestick.getTriggerPressed();
    }

    @Override

    public boolean getClimberClimb()
    {
        return mDrivestick.getRawButtonReleased(17);
    }

    @Override

    public boolean getClimberIdle()
    {
        return mDrivestick.getRawButtonReleased(18);
    }

    @Override
    public boolean getClimberHold()
    {
        return mDrivestick.getRawButtonReleased(19);
    }

    @Override
    public boolean getClimberPrepare()
    {
        return mDrivestick.getRawButtonReleased(20);

    }

    public boolean getScissorLiftOff()
    {
        return mDrivestick.getRawButtonReleased(7);
    }

    public boolean getScissorLiftRetracted()
    {
        return mDrivestick.getRawButtonReleased(8);
    }

    @Override
    public boolean getScissorLiftSwitch()
    {
        return mDrivestick.getRawButtonReleased(9);
    }

    @Override
    public boolean getScissorLiftScale()

    {
        return mDrivestick.getRawButtonReleased(10);
    }

    @Override
    public boolean getScissorLiftManualUp()
    {
        return mDrivestick.getRawButtonReleased(11);
    }

    @Override
    public boolean getScissorLiftManualDown()
    {
        return mDrivestick.getRawButtonReleased(12);
    }

    @Override
    public boolean getHarvesterIntake()
    {
        return mDrivestick.getRawButtonReleased(7); 
    }
    
    @Override
    public boolean getHarvesterEject()
    {
        return mDrivestick.getRawButtonReleased(9); 
    }
    
    @Override
    public boolean getHarvesterOpen()
    {
        return mDrivestick.getRawButtonReleased(11);
    }
    
    public boolean getDebugPrimary()
    {
        return mDrivestick.getRawButton(3);
    }

    @Override
    public boolean getDebugSecondary()
    {
        return mDrivestick.getRawButton(4);
    }

    @Override
    public boolean getDebugTertiary()
    {
        return mDrivestick.getRawButton(5);
    }
}
