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
    private final Joystick mButtonBoard;
    private double previousGetReadyToHarvest;
    private double previousGetDropCube;

    protected ControlBoard()
    {
        mDrivestick = new Joystick(0);
        mButtonBoard = new Joystick(1);
        previousGetReadyToHarvest = 0.0;
        previousGetDropCube = 0.0;
    }

    @Override
    public double getThrottle()
    {
        return -mDrivestick.getZ(); // Is this reversed on the new joystick???
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
    public boolean getReadyToHarvest()
    {
        double current = mButtonBoard.getRawAxis(2);
        boolean isReady;

        isReady = (previousGetReadyToHarvest != current) && (current == 1.0);
        previousGetReadyToHarvest = current;
        return isReady;
    }

    @Override
    public boolean getReadyToDropSwitch()
    {
        return mButtonBoard.getRawButtonPressed(1);
    }

    @Override
    public boolean getReadyToDropScale()
    {
        return mButtonBoard.getRawButtonPressed(2);
    }

    @Override
    public boolean getDropCube()
    {
        double current = mButtonBoard.getRawAxis(3);
        boolean isReady;

        isReady = (previousGetDropCube != current) && (current == 1.0);
        previousGetDropCube = current;
        return isReady;
    }

    @Override
    public boolean getOpenHarvester()
    {
        return mButtonBoard.getRawButtonPressed(5);
    }

    @Override
    public boolean getCloseHarvester()
    {
        return mButtonBoard.getRawButtonPressed(3);
    }

    @Override
    public boolean getEjectCube()
    {
        return mButtonBoard.getRawButtonPressed(4);
    }

    @Override
    public boolean getCarryCube()
    {
        return mButtonBoard.getRawButtonPressed(6);
    }

    @Override
    public boolean getClimb()
    {
        return mButtonBoard.getRawButtonPressed(7);
    }

    @Override
    public boolean getStopClimb()
    {
        return mButtonBoard.getRawButtonPressed(8);
    }

    //Temporary Drive Stick controls
    @Override
    public boolean getClimberIdle()
    {
        return mDrivestick.getRawButtonPressed(5);
    }

    @Override
    public boolean getGrabberTransport()
    {
        return mDrivestick.getRawButtonPressed(6);
    }

    @Override
    public boolean getGrabberGrabCube()
    {
        return mDrivestick.getRawButtonPressed(7);
    }

    @Override
    public boolean getGrabberPrepareDrop()
    {
        return mDrivestick.getRawButtonPressed(8);
    }

    @Override
    public boolean getGrabberPrepareIntake()
    {
        return mDrivestick.getRawButtonPressed(9);
    }
}
