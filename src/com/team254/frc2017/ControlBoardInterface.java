package com.team254.frc2017;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getLowGear();

    // OPERATOR CONTROLS
    boolean getBlinkLEDButton();
}
