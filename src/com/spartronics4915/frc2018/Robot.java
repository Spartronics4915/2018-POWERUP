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
import com.spartronics4915.frc2018.ControlBoardInterface.Sticks;
import com.spartronics4915.frc2018.ControlBoardInterface.Buttons;

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

    // smartdashboard keys
    private static final String kRobotVerbosity = "Robot/Verbosity";
    private static final String kRobotTestModeOptions = "TestModeOptions";
    private static final String kRobotTestMode = "TestMode";
    private static final String kRobotTestVariant = "TestVariant";

    public Robot()
    {
        Logger.logRobotConstruction();
        // please defer initialization of objects until robotInit
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
            SmartDashboard.putString(kRobotVerbosity, "NOTICE"); // competition verbosity

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
            ArrayList<String> canReport = canProbe.getReport();
            Logger.notice("CANDevicesFound: " + canReport);
            int numDevices = canProbe.getCANDeviceCount();
            SmartDashboard.putString("CANBusStatus",
                    numDevices == Constants.kNumCANDevices ? "OK"
                            : ("" + numDevices + "/" + Constants.kNumCANDevices));

            // Subsystem instances
            mDrive = Drive.getInstance();
            mLED = LED.getInstance();
            mGrabber = ArticulatedGrabber.getInstance();
            mClimber = Climber.getInstance();
            mHarvester = Harvester.getInstance();
            mLifter = ScissorLift.getInstance();
            mSuperstructure = Superstructure.getInstance();

            mRobotState = RobotState.getInstance();
            mAutoModeExecuter = null;
            mConnectionMonitor = ConnectionMonitor.getInstance();
            mSubsystemManager = new SubsystemManager(
                    Arrays.asList(mDrive, mSuperstructure,
                            mConnectionMonitor, mLED, mGrabber, mClimber, mHarvester, mLifter));

            // Initialize other helper objects
            mCheesyDriveHelper = new CheesyDriveHelper();
            mControlBoard = new ControlBoard();

            mEnabledLooper = new Looper();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            AutoModeSelector.initAutoModeSelector();
            SmartDashboard.putString(kRobotTestModeOptions,
                    "None,ArticulatedGrabber,Climber,Drive,Harvester,LED,ScissorLift,All");
            SmartDashboard.putString(kRobotTestMode, "None");
            SmartDashboard.putString(kRobotTestVariant, "");

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

    public void zeroAllSensors()
    {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
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
            Logger.setVerbosity(SmartDashboard.getString(kRobotVerbosity, "NOTICE"));
            Logger.logAutoInit();
            Logger.notice("Auto start timestamp: " + Timer.getFPGATimestamp());
            mControlBoard.checkForTestMode();
            if (mAutoModeExecuter != null)
            {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);

            mAutoModeExecuter = null;

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
            Logger.setVerbosity(SmartDashboard.getString(kRobotVerbosity, "NOTICE"));
            Logger.logTeleopInit();
            mControlBoard.checkForTestMode();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            
            mLED.setVisionLampOn();
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
            double throttle = mControlBoard.readStick(Sticks.kThrottle);
            double turn = mControlBoard.readStick(Sticks.kTurn);
            mDrive.setOpenLoop(
                    mCheesyDriveHelper.cheesyDrive(throttle, turn, 
                            mControlBoard.readButton(Buttons.kQuickTurn),
                            !mControlBoard.readButton(Buttons.kLowGear)));

            if (mControlBoard.readButton(Buttons.kReadyToHarvest))
            {
                mLifter.setWantedState(ScissorLift.WantedState.OFF);
            }

            if (mControlBoard.readButton(Buttons.kReadyToDropSwitch))
            {
                mLifter.setWantedState(ScissorLift.WantedState.SWITCH);
            }

            if (mControlBoard.readButton(Buttons.kReadyToDropScale))
            {
                mLifter.setWantedState(ScissorLift.WantedState.SCALE);
            }

            if (mControlBoard.readButton(Buttons.kDropCube))
            {
                //TODO: implement superstructure - see strategy playbook
            }

            if (mControlBoard.readButton(Buttons.kOpenHarvester))
            {
                mHarvester.setWantedState(Harvester.WantedState.OPEN);
            }

            if (mControlBoard.readButton(Buttons.kCloseHarvester))
            {
                mHarvester.setWantedState(Harvester.WantedState.HARVEST);
            }

            if (mControlBoard.readButton(Buttons.kEjectCube))
            {
                mHarvester.setWantedState(Harvester.WantedState.EJECT);
            }

            if (mControlBoard.readButton(Buttons.kCarryCube))
            {
                mSuperstructure.setWantedState(Superstructure.WantedState.TRANSFER_CUBE_TO_GRABBER);
            }

            if (mControlBoard.readButton(Buttons.kClimb))
            {
                mSuperstructure.setWantedState(Superstructure.WantedState.CLIMB);
            }

            if (mControlBoard.readButton(Buttons.kStopClimb))
            {
                mClimber.setWantedState(Climber.WantedState.HOLD);
            }

            if (mControlBoard.readButton(Buttons.kTestClimbIdle))
            {
                mClimber.setWantedState(Climber.WantedState.IDLE);
            }

            if (mControlBoard.readButton(Buttons.kTestGrabberTransport))
            {
                mGrabber.setWantedState(ArticulatedGrabber.WantedState.TRANSPORT);
            }

            if (mControlBoard.readButton(Buttons.kTestGrabberGrabCube))
            {
                mGrabber.setWantedState(ArticulatedGrabber.WantedState.GRAB_CUBE);
            }
            
            if (mControlBoard.readButton(Buttons.kTestGrabberPrepareDrop))
            {
                mGrabber.setWantedState(ArticulatedGrabber.WantedState.PREPARE_DROP);
            }
            
            if (mControlBoard.readButton(Buttons.kTestGrabberPrepareIntake))
            {
                mGrabber.setWantedState(ArticulatedGrabber.WantedState.PREPARE_INTAKE);
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
            Logger.setVerbosity(SmartDashboard.getString(kRobotVerbosity, "NOTICE"));
            Logger.logDisabledInit();
            mControlBoard.checkForTestMode();

            if (mAutoModeExecuter != null)
            {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            zeroAllSensors();
            
            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();
            
            mLED.setVisionLampOff();
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
        // don't zero sensors during disabledPeriodic... zeroAllSensors();
        allButTestPeriodic();
    }

    @Override
    public void testInit()
    {
        Logger.setVerbosity(SmartDashboard.getString(kRobotVerbosity, "NOTICE"));
        String testMode = SmartDashboard.getString(kRobotTestMode, "None");
        String testVariant = SmartDashboard.getString(kRobotTestVariant, "");

        if (testMode.equals("None"))
        {
            Logger.notice("Robot: no tests to run");
            return;
        }
        else
        {
            Logger.notice("Robot: running test mode " + testMode +
                    " variant:" + testVariant + " -------------------------");
            mEnabledLooper.stop();
        }
        Logger.notice("Waiting 5 seconds before running test methods.");
        Timer.delay(5);

        boolean success = true;
        if (testMode.equals("ArticulatedGrabber") || testMode.equals("All"))
        {
            success &= mGrabber.checkSystem(testVariant);
        }

        if (testMode.equals("Drive") || testMode.equals("All"))
        {
            success &= mDrive.checkSystem(testVariant);
        }

        if (testMode.equals("Climber") || testMode.equals("All"))
        {
            success &= mClimber.checkSystem(testVariant);
        }

        if (testMode.equals("Harvester") || testMode.equals("All"))
        {
            success &= mHarvester.checkSystem(testVariant);
        }

        if (testMode.equals("LED") || testMode.equals("All"))
        {
            success &= mLED.checkSystem(testVariant);
        }

        if (testMode.equals("ScissorLift") || testMode.equals("All"))
        {
            success &= mLifter.checkSystem(testVariant);
        }

        if (!success)
        {
            Logger.error("Robot: CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        }
        else
        {
            Logger.notice("Robot: ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic()
    {
        // nothing to do here.
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
