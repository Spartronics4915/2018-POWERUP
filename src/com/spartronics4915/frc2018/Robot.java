package com.spartronics4915.frc2018;

import java.io.IOException;
import java.io.InputStream;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

import com.spartronics4915.frc2018.auto.AutoModeExecuter;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.frc2018.loops.RobotStateEstimator;
import com.spartronics4915.frc2018.loops.VisionProcessor;
import com.spartronics4915.frc2018.paths.profiles.PathAdapter;
import com.spartronics4915.frc2018.subsystems.ArticulatedGrabber;
import com.spartronics4915.frc2018.subsystems.Climber;
import com.spartronics4915.frc2018.subsystems.ConnectionMonitor;
import com.spartronics4915.frc2018.subsystems.Drive;
import com.spartronics4915.frc2018.subsystems.Harvester;
import com.spartronics4915.frc2018.subsystems.LED;
import com.spartronics4915.frc2018.subsystems.ScissorLift;
import com.spartronics4915.frc2018.subsystems.Superstructure;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.CheesyDriveHelper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.DelayedBoolean;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.math.RigidTransform2d;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The main robot class, which instantiates all robot parts and helper classes
 * and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot
 * gets the instance as opposed to creating a
 * new object
 *
 * After initializing all robot parts, the code sets up the autonomous and
 * teleoperated cycles and also code that runs
 * periodically inside both routines.
 *
 * This is the nexus/converging point of the robot code and the best place to
 * start exploring.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this
 * class or the package after creating
 * this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{

    // NB: make sure to construct objects in robotInit, not member declaration,
    //  and usually not constructor.
    private Drive mDrive = null;
    private Superstructure mSuperstructure = null;
    private LED mLED = null;
    private ArticulatedGrabber mGrabber = null;
    private Climber mClimber = null;
    private Harvester mHarvester = null;
    private ScissorLift mLifter = null;
    private RobotState mRobotState = null;
    private AutoModeExecuter mAutoModeExecuter = null;
    private ConnectionMonitor mConnectionMonitor = null;

    // Create subsystem manager
    private SubsystemManager mSubsystemManager = null;

    // Initialize other helper objects
    private CheesyDriveHelper mCheesyDriveHelper = null;
    private ControlBoardInterface mControlBoard = null;

    private Looper mEnabledLooper = null;
    //    private VisionServer mVisionServer = null;
    private AnalogInput mCheckLightButton = null;
    private DelayedBoolean mDelayedAimButton;

    public Robot()
    {
        Logger.logRobotConstruction();
        // please defer initialization of objects until robotInit
    }

    public void zeroAllSensors()
    {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        // Version string and related information
        boolean success = false;
        Logger.logRobotInit();
        try (InputStream manifest =
                getClass().getClassLoader().getResourceAsStream("META-INF/MANIFEST.MF"))
        {
            // build a version string
            Attributes attributes = new Manifest(manifest).getMainAttributes();
            String buildStr = "by: " + attributes.getValue("Built-By") +
                    "  on: " + attributes.getValue("Built-At") +
                    "  (" + attributes.getValue("Code-Version") + ")";
            SmartDashboard.putString("Build", buildStr);

            Logger.notice("=================================================");
            Logger.notice(Instant.now().toString());
            Logger.notice("Built " + buildStr);
            Logger.notice("=================================================");

        }
        catch (IOException e)
        {
            SmartDashboard.putString("Build", "version not found!");
            Logger.warning("Build version not found!");
            DriverStation.reportError(e.getMessage(), false /*
                                                             * no stack trace
                                                             * needed
                                                             */);
        }
        try
        {
            Logger.notice("Robot begin init ------------------");
            // NB: make sure to probe for can devices FIRST since subsystems
            //  may invoke its validate methods.
            CANProbe canProbe = CANProbe.getInstance();
            ArrayList<String> canReport = canProbe.GetReport();
            Logger.notice("CANDevicesFound: " + canReport);
            SmartDashboard.putString("CANBusStatus",
                    canReport.size() == Constants.kNumCANDevices ? "OK"
                            : ("" + canReport.size() + "/" + Constants.kNumCANDevices));

            // Subsystem instances
            mDrive = Drive.getInstance();
            mSuperstructure = Superstructure.getInstance();
            mLED = LED.getInstance();
            mGrabber = ArticulatedGrabber.getInstance();
            mClimber = Climber.getInstance();
            mHarvester = Harvester.getInstance();
            mLifter = ScissorLift.getInstance();
            
            mRobotState = RobotState.getInstance();
            mAutoModeExecuter = null;
            mConnectionMonitor = ConnectionMonitor.getInstance();
            mSubsystemManager = new SubsystemManager(
                    Arrays.asList(mDrive, mSuperstructure,
                            mConnectionMonitor, mLED, mGrabber, mClimber, mHarvester, mLifter));

            // Initialize other helper objects
            mCheesyDriveHelper = new CheesyDriveHelper();
            mControlBoard = new XboxControlBoard();

            mEnabledLooper = new Looper();
            mCheckLightButton = new AnalogInput(Constants.kLEDOnId);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            // mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

            AutoModeSelector.initAutoModeSelector();

            mDelayedAimButton = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
            // Force an true update now to prevent robot from running at start.
            mDelayedAimButton.update(Timer.getFPGATimestamp(), true);

            // Pre calculate the paths we use for auto.
            PathAdapter.calculatePaths();
            zeroAllSensors();
            success = true;
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            // don't throw here, leads to "Robots should not quit..." // throw t;
        }
        Logger.notice("robotInit complete, success:" + success);
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set
     * drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     *
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit()
    {
        try
        {
            Logger.logAutoInit();
            Logger.notice("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null)
            {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);

            mAutoModeExecuter = null;

            mDrive.enableBraking(true);

            mEnabledLooper.start();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();

        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic()
    {
        allButTestPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit()
    {
        try
        {
            Logger.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.enableBraking(false);
            zeroAllSensors();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     *
     * The code uses state machines to ensure that no matter what buttons the
     * driver presses, the robot behaves in a
     * safe and consistent manner.
     *
     * Based on driver input, the code sets a desired state for each subsystem.
     * Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic()
    {
        try
        {
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();
            mDrive.setOpenLoop(
                    mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                            !mControlBoard.getLowGear()));
            
            if (mControlBoard.getBlinkLEDButton())
            {
                mLED.setWantedState(LED.WantedState.BLINK);
            }

            allButTestPeriodic();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit()
    {
        try
        {
            Logger.logDisabledInit();

            if (mAutoModeExecuter != null)
            {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            // leads to Robots should not quit // throw t;
        }
    }

    @Override
    public void disabledPeriodic()
    {
        final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold)
        {
            mLED.setLEDOn();
        }
        else
        {
            mLED.setLEDOff();
        }

        // don't zero sensors during disabledPeriodic... zeroAllSensors();
        allButTestPeriodic();
    }

    @Override
    public void testInit()
    {
        Timer.delay(0.5);

        boolean results = mDrive.checkSystem();
        // e.g. results &= Intake.getInstance().checkSystem();

        if (!results)
        {
            Logger.error("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        }
        else
        {
            Logger.notice("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic()
    {
    }

    /**
     * Helper function that is shared between above periodic functions
     */
    private void allButTestPeriodic()
    {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        mConnectionMonitor.setLastPacketTime(Timer.getFPGATimestamp());
    }

    /**
     * Unused but required function. Plays a similar role to our
     * allPeriodic method. Presumably the timing in IterativeRobotBase wasn't
     * to the liking of initial designers of this system. Perhaps because
     * we don't want it to run during testPeriodic.
     */
    @Override
    public void robotPeriodic()
    {
        // intentionally left blank
    }
}
