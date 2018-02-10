package com.spartronics4915.lib.util.drivers;

import edu.wpi.first.wpilibj.Solenoid;

public class LazySolenoid extends Solenoid
{

    public LazySolenoid(int channel)
    {
        super(channel);
    }
    
    public LazySolenoid(int moduleNumber, int channel)
    {
        super(moduleNumber, channel);
    }
    
    // You can call this method rapidly in a loop
    public void set(boolean state) {
        if (super.get() == state)
            return;
        super.set(state);
    }
    
    public boolean isValid()
    {
        boolean valid = false;
        if(!super.isBlackListed() && 
           !super.getPCMSolenoidVoltageFault())
            valid = true;
        return valid;
    }
}
