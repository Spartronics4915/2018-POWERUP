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

    boolean getClimberClimb();

    boolean getClimberIdle();

    boolean getClimberHold();

    boolean getClimberPrepare();

    boolean getScissorLiftOff();
    
    boolean getScissorLiftRetracted();
    
    boolean getScissorLiftSwitch();
    
    boolean getScissorLiftScale();
    
    boolean getScissorLiftManualUp();
    
    boolean getScissorLiftManualDown();
    
    boolean getHarvesterIntake();
    
    boolean getHarvesterEject();
    
    boolean getHarvesterOpen();
    
    // Debug buttons, remove me
    boolean getDebugPrimary();

    boolean getDebugSecondary();

    boolean getDebugTertiary();

}
