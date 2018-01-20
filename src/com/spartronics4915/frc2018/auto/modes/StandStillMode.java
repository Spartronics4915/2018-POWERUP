package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.lib.util.Logger;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot
 * standstill
 */
public class StandStillMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        Logger.notice("Starting Stand Still Auto Mode... Done!");
    }
}
