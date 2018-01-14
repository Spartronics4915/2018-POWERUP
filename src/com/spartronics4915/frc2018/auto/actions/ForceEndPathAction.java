package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.frc2018.subsystems.Drive;

/**
 * Forces the current path the robot is driving on to end early
 * 
 * @see DrivePathAction
 * @see Action
 * @see RunOnceAction
 */
public class ForceEndPathAction extends RunOnceAction
{

    @Override
    public synchronized void runOnce()
    {
        Drive.getInstance().forceDoneWithPath();
    }
}
