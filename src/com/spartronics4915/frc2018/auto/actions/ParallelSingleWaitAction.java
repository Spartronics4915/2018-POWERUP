package com.spartronics4915.frc2018.auto.actions;

import java.util.List;

public class ParallelSingleWaitAction extends ParallelAction
{
    public ParallelSingleWaitAction(Action... actions)
    {
        super(actions);
    }
    
    public ParallelSingleWaitAction(List<Action> actions)
    {
        super(actions);
    }
    
    @Override
    public boolean isFinished()
    {
        boolean one_finished = false;
        for (Action action : super.mActions)
        {
            if (action.isFinished())
            {
                one_finished = true;
            }
        }
        return one_finished;
    }
}
