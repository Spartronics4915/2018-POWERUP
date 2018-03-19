package com.spartronics4915.frc2018;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface
{
    enum Sticks
    {
        THROTTLE,
        TURN,
    }
    
    enum Buttons
    {
        DRIVE_QUICK_TURN,
        DRIVE_SLOW,
        SCISSOR_OFF,
        SCISSOR_SWITCH,
        SCISSOR_SCALE,
        GRABBER_DROP_CUBE,
        HARVESTER_OPEN,
        HARVESTER_CLOSE,
        HARVESTER_EJECT,
        HARVESTER_PARTIALLY_CLOSE,
        SUPERSTRUCTURE_CARRY_CUBE,
        CLIMBER_TOGGLE,
        GRABBER_FAST_OPEN,
        CAMERA_CHANGE_VIEW,
        CLIMB_IDLE_TEST,
        GRABBER_TRANSPORT_TEST,
        GRABBER_GRAB_CUBE_TEST,
        GRABBER_PREPARE_DROP_TEST,
        GRABBER_PREPARE_INTAKE_TEST,
        VISION_CUBE_HARVEST,
        GRABBER_TOGGLE,
        GRABBER_TEMP_TEST
    }
    
    double readStick(Sticks a);
    boolean readButton(Buttons b);
    void checkForTestMode();

}
