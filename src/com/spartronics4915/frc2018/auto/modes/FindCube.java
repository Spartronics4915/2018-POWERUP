package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.FindCubeAction;

public class FindCube extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(new FindCubeAction());
    }

}
