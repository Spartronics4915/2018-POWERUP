package com.spartronics4915.lib.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class LazyDoubleSolenoid extends DoubleSolenoid
{
    public LazyDoubleSolenoid(int forwardChannel, int reverseChannel)
    {
        super(forwardChannel, reverseChannel);
    }
    
    public LazyDoubleSolenoid(int moduleNumber, int forwardChannel, int reverseChannel)
    {
        super(moduleNumber, forwardChannel, reverseChannel);
    }
    
    public void set(Value value) {
        if (super.get() == value)
            return;
        super.set(value);
    }
    
    public boolean isValid()
    {
        boolean valid = false;
        if (!super.isFwdSolenoidBlackListed() && !super.isRevSolenoidBlackListed() &&
                !super.getPCMSolenoidVoltageFault())
            valid = true;
        return valid;
    }
}
