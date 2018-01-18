package com.spartronics4915.frc2018.auto.actions;

import com.spartronics4915.lib.util.Logger;

/**
 * Prints a message to the console for debug purposes
 * 
 * @see Action
 * @see RunOnceAction
 */
public class PrintDebugAction extends RunOnceAction implements Action
{

    String debugMessage;

    public PrintDebugAction(String s)
    {
        debugMessage = s;
    }

    @Override
    public void runOnce()
    {
        Logger.notice("PrintDebugAction:" + debugMessage);
    }

}
