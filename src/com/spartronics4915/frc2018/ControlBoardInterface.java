package com.spartronics4915.frc2018;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface
{
    enum Sticks
    {
        kThrottle,
        kTurn,
    }
    
    enum Buttons
    {
        kQuickTurn,
        kLowGear,
        kReadyToHarvest,
        kReadyToDropSwitch,
        kReadyToDropScale,
        kDropCube,
        kOpenHarvester,
        kCloseHarvester,
        kEjectCube,
        kCarryCube,
        kClimb,
        kStopClimb,
        kTestClimbIdle,
        kTestGrabberTransport,
        kTestGrabberGrabCube,
        kTestGrabberPrepareDrop,
        kTestGrabberPrepareIntake
    }
    
    double readStick(Sticks a);
    boolean readButton(Buttons b);
    void checkForTestMode();

}
