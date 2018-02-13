package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Subsystem abstract class serves as a basic framework for all robot
 * subsystems.  Subsystems only have one instance (after all, one robot does not have two
 * drivetrains). Subsystems typically embody a state machine with a desired-state and 
 * actual-state. To change the subsystem state clients must get the instance of the 
 * subsystem and request a state change by establishing a new desired-state..
 * Robot code will try to match the two states by actuating devices, etc.
 * 
 * This base class offers standard logging methods for subclasses.  It also
 * stores the initialization success.
 * 
 * Subsystem subclasses are responsible for:
 *      - implementing Singleton behavior (via getInstance() pattern).
 *      - initializing all member components during construction
 *      - invoking logInitialized with success/failure at the end of construction
 *      - behaving in a reasonable manner if initialization isn't successful.
 *
 * Each subsystem must implement these methods:
 *      - outputToSmartDashboard
 *      - stop() has a stop routine (for after each match), 
 *      - zeroSensors() - which helps with calibration.
 *      - registerEnabledLoops() -    
 *
 */
public abstract class Subsystem
{
    // all subsystems should set mInitialized upon successful init.
    private boolean mInitialized = false;
    private String mName = null;
    public boolean isInitialized() { return mInitialized; }

    
    protected Subsystem() // means only subclasses can exist, not Subsystems
    {        
        String classname = this.getClass().getName();
        int tail = classname.lastIndexOf('.');
        if(tail == -1)
        {
            mName = classname;
        }
        else
        {
            mName = classname.substring(tail+1);
        }   
    }
    
    public String getName()
    {
        return mName;
    }
    
    public void logInitialized(boolean success)
    {
        mInitialized = success;
        if(success)
            this.logNotice("init SUCCEEDED");
        else
            this.logWarning("init FAILED");
        SmartDashboard.putString(mName+"/Status", mInitialized ? "OK" : "ERROR");
    }
    
    // broadcast methods are for smartdashboard with conventionalized keys
    public void dashboardPutState(String state)
    {
        SmartDashboard.putString(mName+"/State", state);
    }
    
    public void dashboardPutWantedState(String state)
    {
        SmartDashboard.putString(mName+"/WantedState", state);
    }
   
    public void dashboardPutString(String nm, String value)
    {
        SmartDashboard.putString(mName+"/" + nm, value);
    }
    
    public String dashboardGetString(String nm, String defValue)
    {
        return SmartDashboard.getString(mName+"/" + nm, defValue);
    }
    
    public void dashboardPutNumber(String nm, Number value)
    {
        SmartDashboard.putNumber(mName+"/"+nm, value.doubleValue());
    }
    
    public Number dashboardGetNumber(String nm, Number defaultValue)
    {
        return SmartDashboard.getNumber(mName+"/"+nm, defaultValue.doubleValue());
    }
    
    public void dashboardPutBoolean(String nm, Boolean value)
    {
        SmartDashboard.putBoolean(mName+"/"+nm, value);
    }

    public boolean dashboardGetBoolean(String nm, Boolean defValue)
    {
        return SmartDashboard.getBoolean(mName+"/"+nm, defValue);
    }

    // log methods are for conventionalizing format across subsystems 
    public void logException(String msg, Throwable e)
    {
        Logger.logThrowableCrash(this.getName() + " " + msg, e);
    }
    
    public void logError(String msg)
    {
        Logger.error(this.getName() + " " + msg); 
    }
    
    public void logWarning(String msg)
    {
        Logger.warning(this.getName() + " " + msg); 
    }
    
    public void logNotice(String msg)
    {
        Logger.notice(this.getName() + " " + msg); 
    }
    
    public void logInfo(String msg)
    {
        Logger.info(this.getName() + " " + msg); 
    }
    
    public void logDebug(String msg)
    {
        Logger.debug(this.getName() + " " + msg); 
    }
    
    public void writeToLog()
    {
        /* an optional override... SubsystemManager invokes this */
    }
    
    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);
    
    public abstract boolean checkSystem(String variant);
    
}
