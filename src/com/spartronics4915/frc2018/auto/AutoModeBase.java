package com.spartronics4915.frc2018.auto;

import com.spartronics4915.frc2018.auto.actions.Action;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Util;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase
{

    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run()
    {
        m_active = true;
        try
        {
            Logger.notice("Game specific message: " + Util.getGameSpecificMessage());
            routine();
        }
        catch (AutoModeEndedException e)
        {
            Logger.notice("Auto mode ending early");
            return;
        }

        done();
        Logger.notice("Auto mode done");
    }

    public void done()
    {
    }

    public void stop()
    {
        m_active = false;
    }

    public boolean isActive()
    {
        return m_active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException
    {
        if (!isActive())
        {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException
    {
        isActiveWithThrow();
        action.start();

        while (isActiveWithThrow() && !action.isFinished())
        {
            action.update();
            long waitTime = (long) (m_update_rate * 1000.0);

            try
            {
                Thread.sleep(waitTime);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }

        action.done();
    }

}
