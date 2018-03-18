package com.spartronics4915.frc2018.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions
 * report being done.
 * 
 * @param A
 *        List of Action objects
 */
public class ParallelAction implements Action
{

    private final List<Action> mActions;

    public ParallelAction(Action... actions)
    {
        this(Arrays.asList(actions));
    }
    
    public ParallelAction(List<Action> actions)
    {
        mActions = new ArrayList<Action>(actions);
    }

    @Override
    public boolean isFinished()
    {
        boolean all_finished = true;
        for (Action action : mActions)
        {
            if (!action.isFinished())
            {
                all_finished = false;
            }
        }
        return all_finished;
    }

    @Override
    public void update()
    {
        for (Action action : mActions)
        {
            action.update();
        }
    }

    @Override
    public void done()
    {
        for (Action action : mActions)
        {
            action.done();
        }
    }

    @Override
    public void start()
    {
        for (Action action : mActions)
        {
            action.start();
        }
    }
}
