package com.spartronics4915.frc2018.auto.modes;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.AutoModeEndedException;
import com.spartronics4915.frc2018.auto.actions.ExampleAction;

public class ExampleMode extends AutoModeBase
{

    // To make your auto mode be seen on the Dashboard, you need to add it
    // in com.spartronics4915.frc2018.AutoModeSelector.
    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(new ExampleAction());
    }

}
