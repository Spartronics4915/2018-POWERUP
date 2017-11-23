package org.usfirst.frc.team4915.powerup.subsystems;

import org.usfirst.frc.team4915.powerup.Logger;
import org.usfirst.frc.team4915.powerup.commands.StopCommand;

/**
 * We're doing port and starboard again. That's all that really matters here.
 * These directions are relative to the front of the robot, which removes as 
 * much possible ambiguity as you can with directions (in this context).
 * Port and starboard refer respectively to left and right, relative to the
 * front of the robot.
 */
public class Drivetrain extends SpartronicsSubsystem {

    // This is public to stop logger proliferation by giving Commands a logger
    public Logger m_logger;
    
    public Drivetrain()
    {
        m_logger = new Logger("Drivetrain", Logger.Level.DEBUG);
        
        // Pretty much everything should go here, especially because
        // certain initializations can throw exceptions.
        try
        {
            
            // This needs to go at the end. We only set m_initialized
            // (a field of the parent class) on failure (to false).
            m_logger.info("initialized successfully");
        }
        catch (Exception e)
        {
            m_logger.exception(e, false);
            m_initialized = false;
        }
    }
    
    public void initDefaultCommand()
    {
        if (initialized())
        {
            setDefaultCommand(new StopCommand(this));
        }
    }

    public void stop()
    {
        // FIXME: Actually stop the motors
    }
}

