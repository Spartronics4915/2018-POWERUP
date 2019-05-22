package com.spartronics4915.frc2018;

import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private static final boolean kUseBackupDrivestick = false;

    public static ControlBoardInterface getInstance()
    {
        if (mInstance == null)
        {
            if(kUseBackupDrivestick)
            {
                mInstance = new BackupControlBoard();
            }
            else
            {
                mInstance = new ControlBoard();
            }
        }
        return mInstance;
    }

    private final Joystick mDrivestick;
    private final Joystick mButtonBoard;
    private double mPreviousGetReadyToHarvest;
    private double mPreviousSwitch;
    private double mPreviousDeploy;
    private double mPreviousSlideDrop;
    private double mPreviousTransport;
    private boolean mTestsAllowed;

    protected ControlBoard()
    {
        mDrivestick = new Joystick(0);
        mButtonBoard = new Joystick(1);
        mPreviousGetReadyToHarvest = 0.0;
        mPreviousSwitch = 0.0;
        mPreviousDeploy = 0.0;
        mPreviousTransport = 0.0;
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
                result = -mDrivestick.getY();
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
            case DRIVE_SLOW:
                result = mDrivestick.getRawButton(1);
                break;
            case SCISSOR_OFF:
                current = mButtonBoard.getRawAxis(2);
                result = (mPreviousGetReadyToHarvest != current) && (current == 1.0);
                mPreviousGetReadyToHarvest = current;
                break;
            case SCISSOR_SWITCH:
                current = mButtonBoard.getRawAxis(3);
                result = (mPreviousSwitch != current) && (current == 1.0);
                mPreviousSwitch = current;
                break;
            case SCISSOR_LOW_SCALE:
                result = mButtonBoard.getRawButtonPressed(1);
                break;
            case SCISSOR_SCALE:
                result = mButtonBoard.getRawButtonPressed(2);
                break;
            case HARVESTER_EJECT:
                result = mButtonBoard.getRawButtonPressed(4);
                break;
            case CLIMBER_TOGGLE:
                result = mButtonBoard.getRawButtonPressed(7);
                break;
            case CAMERA_CHANGE_VIEW:
                result = mButtonBoard.getRawButtonPressed(10) || mDrivestick.getRawButtonPressed(2);
                break;
            case HARVESTER_GRAB:
                result = mButtonBoard.getRawButtonPressed(3);
                break;
            case HARVESTER_INTAKE:
                result = mButtonBoard.getRawButtonPressed(5);
                break;
            case HARVESTER_FLOAT:
                result = mButtonBoard.getRawButtonPressed(9);
                break;
            case HARVESTER_OPEN:
                result = mButtonBoard.getRawButtonPressed(6);
                break;
            case HARVESTER_SLIDE_DROP:
                current = mButtonBoard.getRawAxis(1);
                result = (mPreviousSlideDrop != current) && (current == -1.0);
                mPreviousSlideDrop = current;
                break;
            case HARVESTER_DEPLOY:
                current = mButtonBoard.getRawAxis(1);
                result = (mPreviousDeploy != current) && (current == 1.0);
                mPreviousDeploy = current;
                break;
            case VISION_CUBE_HARVEST:
                // Teleop Harvest Cubes
                result = mDrivestick.getRawButton(4);
                break;
            default:
                Logger.error("ControlBoard: unimplemented boolean: " + b.toString());
                break;
        }
        return result;
    }
}
