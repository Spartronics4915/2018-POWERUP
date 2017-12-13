package org.usfirst.frc.team4915.powerup.commands;

import org.usfirst.frc.team4915.powerup.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command is to keep motors safety happy, and is also an example of the
 * boilerplate/logger type code you need for a command.
 */
public class StopCommand extends Command
{

    private Drivetrain m_drivetrain;

    public StopCommand(Drivetrain drivetrain)
    {
        m_drivetrain = drivetrain;
        requires(drivetrain);
    }

    protected void initialize()
    {
        m_drivetrain.m_logger.info("StopCommand initialized");
    }

    protected void execute()
    {
        m_drivetrain.stop();
    }

    protected boolean isFinished()
    {
        return false;
    }

    protected void end()
    {
        m_drivetrain.m_logger.info("StopCommand ended");
    }

    protected void interrupted()
    {
        m_drivetrain.m_logger.info("StopCommand interrupted");
    }
}
