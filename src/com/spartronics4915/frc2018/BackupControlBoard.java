package com.spartronics4915.frc2018;

import com.spartronics4915.frc2018.ControlBoardInterface.Buttons;
import com.spartronics4915.frc2018.ControlBoardInterface.Sticks;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains the button mappings for the backup drive stick 
 * and the mechanism controller.
 * A singleton.
 * Implements the ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */

public class BackupControlBoard implements ControlBoardInterface
{
    private final Joystick mDrivestick;
    private final Joystick mButtonBoard;
    private double mPreviousGetReadyToHarvest;
    private double mPreviousGetDropCube;
    private boolean mTestsAllowed;

    protected BackupControlBoard()
    {
        mDrivestick = new Joystick(0);
        mButtonBoard = new Joystick(1);
        mPreviousGetReadyToHarvest = 0.0;
        mPreviousGetDropCube = 0.0;
        checkForTestMode();
    }
    
    public void checkForTestMode()
    {
        mTestsAllowed = SmartDashboard.getBoolean("TestingGUI", false);
    }

    @Override
    public double readStick(Sticks a)
    {
        double result = 0.;
        switch (a)
        {
            case THROTTLE:
                result = -mDrivestick.getZ();
                break;
            case TURN:
                result = mDrivestick.getX();
                break;
            default:
                Logger.error("ControlBoard: unimplemented analog control: " + a.toString());
                break;
        }
        return result;
    }

    @Override
    public boolean readButton(Buttons b)
    {
        boolean result = false;
        double current;
        switch (b)
        {
            case DRIVE_QUICK_TURN:
                result = mDrivestick.getRawButtonPressed(1);
                break;
            case DRIVE_SLOW:
                result = mDrivestick.getTriggerPressed(); // available!
                break;
            case SCISSOR_OFF:
                current = mButtonBoard.getRawAxis(2);
                result = (mPreviousGetReadyToHarvest != current) && (current == 1.0);
                mPreviousGetReadyToHarvest = current;
                break;
            case SCISSOR_SWITCH:
                result = mButtonBoard.getRawButtonPressed(1);
                break;
            case SCISSOR_SCALE:
                result = mButtonBoard.getRawButtonPressed(2);
                break;
            case GRABBER_DROP_CUBE:
                current = mButtonBoard.getRawAxis(3);
                result = (mPreviousGetDropCube != current) && (current == 1.0 || mDrivestick.getRawButtonPressed(6));
                mPreviousGetDropCube = current;
                break;
            case HARVESTER_OPEN:
                result = mButtonBoard.getRawButtonPressed(5) || mDrivestick.getRawButtonPressed(2);
                break;
            case HARVESTER_CLOSE:
                result = mButtonBoard.getRawButtonPressed(3) || mDrivestick.getRawButtonPressed(3);
                break;
            case HARVESTER_EJECT:
                result = mButtonBoard.getRawButtonPressed(4);
                break;
            case SUPERSTRUCTURE_CARRY_CUBE:
                result = mButtonBoard.getRawButtonPressed(6) || mDrivestick.getRawButtonPressed(5);
                break;
            case CLIMBER_TOGGLE:
                result = mButtonBoard.getRawButtonPressed(7);
                break;
            case CAMERA_CHANGE_VIEW:
                result = mButtonBoard.getRawButtonPressed(10) || mDrivestick.getRawButtonPressed(7);
                break;
            case GRABBER_TRANSPORT_TEST:
                result = mDrivestick.getRawButtonPressed(9);
                break;
            case CLIMB_IDLE_TEST:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(8) : false;
                break;
            case GRABBER_TEMP_TEST:
                break;
            case GRABBER_GRAB_CUBE_TEST:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(10) : false;
                break;
            case GRABBER_PREPARE_DROP_TEST:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(11) : false;
                break;
            case GRABBER_PREPARE_INTAKE_TEST:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(12) : false;
                break;
            case VISION_CUBE_HARVEST:
                // Teleop Harvest Cubes
                result = mDrivestick.getRawButtonPressed(4);
                break;
            default:
                Logger.error("ControlBoard: unimplemented boolean: " + b.toString());
                break;
        }
        return result;
    }

}
