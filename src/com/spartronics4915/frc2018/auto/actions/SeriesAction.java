package com.spartronics4915.frc2018.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action
{

    private Action mCurAction;
    private final List<Action> mRemainingActions;

    public SeriesAction(Action... actions)
    {
        this(Arrays.asList(actions));
    }
    
    public SeriesAction(List<Action> actions)
    {
        mRemainingActions = new ArrayList<Action>(actions);

        mCurAction = null;
    }

    @Override
    public boolean isFinished()
    {
        return mRemainingActions.isEmpty() && mCurAction == null;
    }

    @Override
    public void start()
    {
    }

    @Override
    public void update()
    {
        if (mCurAction == null)
        {
            if (mRemainingActions.isEmpty())
            {
                return;
            }

            mCurAction = mRemainingActions.remove(0);
            mCurAction.start();
        }

        mCurAction.update();

        if (mCurAction.isFinished())
        {
            mCurAction.done();
            mCurAction = null;
        }
    }

    @Override
    public void done()
    {
    }
}
