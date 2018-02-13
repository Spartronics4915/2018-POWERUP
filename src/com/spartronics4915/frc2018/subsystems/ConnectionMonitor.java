package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.LatchedBoolean;
import com.spartronics4915.lib.util.Logger;

/**
 * Keeps track of the robot's connection to the driver station. If it
 * disconnects for more than 1 second, start blinking
 * the LEDs.
 */
public class ConnectionMonitor extends Subsystem
{

    public static double kConnectionTimeoutSec = 1.0;

    private static ConnectionMonitor mInstance = null;

    public static ConnectionMonitor getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new ConnectionMonitor();
        }
        return mInstance;
    }

    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected;
    private LatchedBoolean mJustDisconnected;
    private LED mLED;

    ConnectionMonitor()
    {
        mLastPacketTime = 0.0;
        mJustReconnected = new LatchedBoolean();
        mJustDisconnected = new LatchedBoolean();
        mLED = LED.getInstance();
        logInitialized(true);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(new Loop()
        {

            @Override
            public void onStart(double timestamp)
            {
                synchronized (ConnectionMonitor.this)
                {
                    mLastPacketTime = timestamp;
                }
            }

            @Override
            public void onLoop(double timestamp)
            {
                synchronized (ConnectionMonitor.this)
                {
                    boolean has_connection = true;
                    if (timestamp - mLastPacketTime > kConnectionTimeoutSec)
                    {
                        String msg = "ConnectionMonitor: connection lost or timed out.";
                        Logger.warning(msg);
                        mLED.warnDriver(msg);
                        has_connection = false;
                    }

                    if (mJustReconnected.update(has_connection))
                    {
                        // Reconfigure blink if we are just connected.
                        mLED.configureBlink(LED.kDefaultBlinkCount, LED.kDefaultBlinkDuration);
                    }

                    if (mJustDisconnected.update(!has_connection))
                    {
                        // Reconfigure blink if we are just disconnected.
                        mLED.configureBlink(LED.kDefaultBlinkCount, LED.kDefaultBlinkDuration * 2.0);
                    }
                }
            }

            @Override
            public void onStop(double timestamp)
            {

            }
        });
    }

    @Override
    public void outputToSmartDashboard()
    {
    }

    @Override
    public void stop()
    {

    }

    @Override
    public void zeroSensors()
    {

    }
    
    @Override
    public boolean checkSystem(String variant)
    {
        logNotice("checkSystem (unimplemented) -----------");
        return true;
    }

    public synchronized void setLastPacketTime(double timestamp)
    {
        mLastPacketTime = timestamp;
    }
}
