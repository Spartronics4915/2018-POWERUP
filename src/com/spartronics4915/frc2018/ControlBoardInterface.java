package com.spartronics4915.frc2018;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface
{

    // DRIVER CONTROLS
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getLowGear();

    //MECHANISM CONTROLS
    boolean getReadyToHarvest();

    boolean getReadyToDropSwitch();

    boolean getReadyToDropScale();

    boolean getDropCube();

    boolean getOpenHarvester();

    boolean getCloseHarvester();

    boolean getEjectCube();

    boolean getCarryCube();

    boolean getClimb();

    boolean getStopClimb();

    //TESTING CONTROLS
    boolean getClimberIdle();

    boolean getGrabberTransport();

    boolean getGrabberGrabCube();

    boolean getGrabberPrepareDrop();

    boolean getGrabberPrepareIntake();
}
