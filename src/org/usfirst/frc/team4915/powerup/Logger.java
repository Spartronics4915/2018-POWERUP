package org.usfirst.frc.team4915.powerup;

import java.util.ArrayList;
import java.util.List;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

// Logger:
//  a simple class to route all logging through.
//  future enhancement:
//      * support logging to file
//      * runtime inference of logging level (practice vs competition)
//  usage:
//      singleton: Logger.getInstance().debug("here's my message");
//      via Robot:  robot.m_logger.debug("here's a debugging msg");
//      via Subsystem: this.m_logger.debug("here's a message");
//  loglevel conventions:
//      debug: used for debugging software... not available during
//          competition.
//      info:  used for less interesting but non-debugging message.
//      notice: used for msgs you always want to see in log
//          - subsystem initializations
//          - important state transitions (ie entering auto, teleop)
//      warning: used to convey non-fatal but abnormal state
//      error: used to convey strong abnormal conditions
//      exception: used in a catch block to report exceptions.
//
public class Logger
{

    // The code formatter attempts to reorder the levels, that doesn't help...

    //@formatter:off
    public enum Level
    {
        DEBUG,
        INFO,
        NOTICE,
        WARNING,
        ERROR
    }
    //@formatter:on

    private static List<Logger> s_allLoggers = new ArrayList<>();
    private static Logger s_sharedLogger;
    private static DateFormat s_dateFormat = new SimpleDateFormat("hh:mm:ss");
    // NB: changing the date-format may negatively impact downstream/display
    //     code that may depend upon this format.

    private Level m_loglevel; // per-instance
    private String m_namespace;

    public static List<Logger> getAllLoggers()
    {
        return s_allLoggers;
    }

    public static Logger getSharedInstance()
    {
        if (s_sharedLogger == null)
        {
            s_sharedLogger = new Logger("<shared>", Level.DEBUG);
        }
        return s_sharedLogger;
    }

    public Logger(String nm, Level lev)
    {
        m_namespace = nm;
        m_loglevel = lev;
        s_allLoggers.add(this);
    }

    public void debug(String msg)
    {
        if (reportLevel(Level.DEBUG))
        {
            logMsg("DEBUG  ", msg);
        }
    }

    public void info(String msg)
    {
        if (reportLevel(Level.INFO))
        {
            logMsg("INFO   ", msg);
        }
    }

    public void notice(String msg)
    {
        if (reportLevel(Level.NOTICE))
        {
            logMsg("NOTICE ", msg);
        }
    }

    public void warning(String msg)
    {
        if (reportLevel(Level.WARNING))
        {
            logMsg("WARNING", msg);
        }
    }

    public void error(String msg)
    {
        logMsg("ERROR  ", msg);
    }

    public void exception(Exception e, boolean skipStackTrace)
    {
        logMsg("EXCEPT ", e.getMessage());
        if (!skipStackTrace)
        {
            e.printStackTrace();
        }
    }

    private void logMsg(String lvl, String msg)
    {
        Date now = new Date();
        String nowstr = s_dateFormat.format(now);
        System.out.println(nowstr + " " + lvl + " " + m_namespace + ": " + msg + "\r"); // We need \r because National Instruments likes to compress frequent messages without the carriage return
        // NB: changing the date-format may negatively impact downstream/display
        //     code that may depend upon this format.
    }

    private boolean reportLevel(Level lev)
    {
        return lev.ordinal() >= m_loglevel.ordinal();
    }

    public String getNamespace()
    {
        return m_namespace;
    }

    public Level getLogLevel()
    {
        return m_loglevel;
    }

    public void setLogLevel(Level level)
    {
        m_loglevel = level;
    }
}
