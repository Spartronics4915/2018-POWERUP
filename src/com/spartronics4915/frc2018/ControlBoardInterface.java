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
        DRIVE_SLOW,
        SCISSOR_OFF,
        SCISSOR_SWITCH,
        SCISSOR_LOW_SCALE,
        SCISSOR_SCALE,
        GRABBER_SLIDE_DROP_CUBE,
        HARVESTER_GRAB,
        HARVESTER_INTAKE,
        HARVESTER_FLOAT,
        HARVESTER_EJECT,
        HARVESTER_SLIDE_DROP,
        HARVESTER_DEPLOY,
        HARVESTER_OPEN,
        CLIMBER_TOGGLE,
        CAMERA_CHANGE_VIEW,
        VISION_CUBE_HARVEST,
    }

    double readStick(Sticks a);
    boolean readButton(Buttons b);
    void checkForTestMode();

}
