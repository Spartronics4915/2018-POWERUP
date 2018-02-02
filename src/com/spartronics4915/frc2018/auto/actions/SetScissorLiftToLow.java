package com.spartronics4915.frc2018.auto.actions;


import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.lib.util.Logger;
/**
 * Action Lowers Scissor Lift to Low Height
 * 
 * Action Interface, an interface that describes an iterative action. It is run
 * by an autonomous action, called by the
 * method runAction in AutoModeBase (or more commonly in autonomous modes that
 * extend AutoModeBase)
 *
 * @see com.spartronics4915.frc2018.auto.AutoModeBase#runAction
 */
public class SetScissorLiftToLow implements Action
{

    
    private ScissorLift mScissorLift = ScissorLift.getInstance();
    /**
     * Returns whether or not the code has finished execution. When implementing
     * this interface, this method is used by
     * the runAction method every cycle to know when to stop running the action
     * 
     * @return boolean
     */
    public boolean isFinished()
    {
        return mScissorLift.isDone();
    }

    /**
     * Called by runAction in AutoModeBase iteratively until isFinished returns
     * true. Iterative logic lives in this
     * method
     */
    public void update()
    {
        
    }

    /**
     * Run code once when the action finishes, usually for clean up
     */
    public void done()
    {
        Logger.notice("SetScissorLiftToLow is Done");
    }
    /**
     * Run code once when the action is started, for set up
     */
    public void start()
    {
        Logger.notice("SetScissorLiftToLow is Starting");
        
    }
    
    
}
