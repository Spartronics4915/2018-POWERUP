package org.usfirst.frc.team4915.powerup.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

// SpartronicsSubsystem is intended to represent the base
// class for all subsystems.  Its current purpose is to
// capture whether the subsystem has been fully/properly
// initialized.  During development, incomplete/flaky hardware
// implementations can make our software unstable and this
// class is an attempt to remedy this.   
//
// We require subclasses to:
//   1) set the protected field, m_initialized, to false if init fails.
//   2) protect all (private) code that depends upon proper initialization
//      within if(initialized() {} blocks.
// Now commands can merrily run along without awareness of proper
// initialization without throwing exceptions that produce the
// dreaded "Robot's don't quit" message.
//
// Gotcha:  while this does alleviate problems of spurious exceptions
//   it doesn't solve the problem of commands that require feedback
//   from a subsystem for progress or completion.

public abstract class SpartronicsSubsystem extends Subsystem
{

    protected boolean m_initialized = true;

    public boolean initialized()
    {
        return m_initialized;
    }

    abstract void validate();
}
