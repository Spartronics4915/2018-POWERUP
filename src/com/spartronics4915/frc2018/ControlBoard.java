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
    private double mPreviousGetReadyToHarvest;
    private double mPreviousGetDropCube;
    private boolean mTestsAllowed;

    protected ControlBoard()
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
            case kThrottle:
                result = -mDrivestick.getZ(); // Is this reversed on the new joystick???
                break;
            case kTurn:
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
            case kQuickTurn:
                result = mDrivestick.getRawButtonPressed(2);
                break;
            case kLowGear:
                result = mDrivestick.getTriggerPressed(); // available!
                break;
            case kReadyToHarvest:
                current = mButtonBoard.getRawAxis(2);
                result = (mPreviousGetReadyToHarvest != current) && (current == 1.0);
                mPreviousGetReadyToHarvest = current;
                break;
            case kReadyToDropSwitch:
                result = mButtonBoard.getRawButtonPressed(1);
                break;
            case kReadyToDropScale:
                result = mButtonBoard.getRawButtonPressed(2);
                break;
            case kDropCube:
                current = mButtonBoard.getRawAxis(3);
                result = (mPreviousGetDropCube != current) && (current == 1.0);
                mPreviousGetDropCube = current;
                break;
            case kOpenHarvester:
                result = mButtonBoard.getRawButtonPressed(5);
                break;
            case kCloseHarvester:
                result = mButtonBoard.getRawButtonPressed(3);
                break;
            case kEjectCube:
                result = mButtonBoard.getRawButtonPressed(4);
                break;
            case kCarryCube:
                result = mButtonBoard.getRawButtonPressed(6);
                break;
            case kClimb:
                result = mButtonBoard.getRawButtonPressed(7);
                break;
            case kStopClimb:
                result = mButtonBoard.getRawButtonPressed(8);
                break;
            case kTestClimbIdle:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(5) : false;
                break;
            case kTestGrabberTransport:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(6) : false;
                break;
            case kTestGrabberGrabCube:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(7) : false;
                break;
            case kTestGrabberPrepareDrop:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(8) : false;
                break;
            case kTestGrabberPrepareIntake:
                result = mTestsAllowed ? mDrivestick.getRawButtonPressed(9) : false;
                break;
            default:
                Logger.error("ControlBoard: unimplemented boolean: " + b.toString());
                break;
        }
        return result;
    }
}
