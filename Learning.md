## Learning Spartronics-254Base

<!-- TOC depthFrom:3 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [Basic execution model, bootstrapping, debugging](#basic-execution-model-bootstrapping-debugging)
	- [how does our code get built, downloaded and launched?](#how-does-our-code-get-built-downloaded-and-launched)
	- [where are crashlogs found?  How did they get there?](#where-are-crashlogs-found-how-did-they-get-there)
	- [where is java main? When does our code get control?](#where-is-java-main-when-does-our-code-get-control)
	- [what is the `Looper`.  When is it created? When is it started/stopped?](#what-is-the-looper-when-is-it-created-when-is-it-startedstopped)
	- [what does `registerEnabledLoops()` do?  Who calls it?](#what-does-registerenabledloops-do-who-calls-it)
- [Threads](#threads)
	- [what is a `Thread`?](#what-is-a-thread)
	- [what is a `Runnable`,  `CrashTrackingRunnable`?](#what-is-a-runnable-crashtrackingrunnable)
	- [what guarantees does Thread.sleep() offer?  How many invocations of this](#what-guarantees-does-threadsleep-offer-how-many-invocations-of-this)
- [Commands, Scheduler](#commands-scheduler)
	- [how is the autonomous mode selected?  How does the selected mode receive](#how-is-the-autonomous-mode-selected-how-does-the-selected-mode-receive)
	- [how does the autonomous mode control subsystems? What’s the difference](#how-does-the-autonomous-mode-control-subsystems-whats-the-difference)
	- [what is the frequency of the scheduler loops, where is this embodied?](#what-is-the-frequency-of-the-scheduler-loops-where-is-this-embodied)
	- [how many threads are running at the same time in auto and tele?](#how-many-threads-are-running-at-the-same-time-in-auto-and-tele)
	- [what is a “wantedState” as specified in Robot::teleopPeriodic?](#what-is-a-wantedstate-as-specified-in-robotteleopperiodic)
- [Subsystem](#subsystem)
	- [how many subsystems are there? Do they share a base class?](#how-many-subsystems-are-there-do-they-share-a-base-class)
	- [is there a single database of motor identifiers, gpio pins, etc?](#is-there-a-single-database-of-motor-identifiers-gpio-pins-etc)
	- [can subsystems modify the state of other subsystems?](#can-subsystems-modify-the-state-of-other-subsystems)
	- [what is the purpose of `SubsystemManager`?](#what-is-the-purpose-of-subsystemmanager)
	- [LED](#led)
		- [when/how does the LED subsystem determine what blinking pattern to emit?](#whenhow-does-the-led-subsystem-determine-what-blinking-pattern-to-emit)
	- [Superstructure](#superstructure)
		- [how is this subsystem different from others?](#how-is-this-subsystem-different-from-others)
	- [Drive](#drive)
		- [How many drive motors? What is a follower motor?](#how-many-drive-motors-what-is-a-follower-motor)
		- [How are encoders handled?  How are encoder ticks converted to user-units?](#how-are-encoders-handled-how-are-encoder-ticks-converted-to-user-units)
		- [What is CheesyDrive?  How does it operate?](#what-is-cheesydrive-how-does-it-operate)
		- [In teleop, are we operating in setVelocity or in PercentOutput mode?](#in-teleop-are-we-operating-in-setvelocity-or-in-percentoutput-mode)
		- [When do they use positionControlMode?](#when-do-they-use-positioncontrolmode)
		- [What units do they use in velocity control mode?](#what-units-do-they-use-in-velocity-control-mode)
		- [How many internal states does the Drive subsystem have? What do they do?](#how-many-internal-states-does-the-drive-subsystem-have-what-do-they-do)
		- [Do any internal states lead automatically to other internal states?](#do-any-internal-states-lead-automatically-to-other-internal-states)
		- [Are MotorSafety settings in play?  If so, how do we prevent](#are-motorsafety-settings-in-play-if-so-how-do-we-prevent)
		- [What’s the relationship between Drive and RobotStateEstimator?](#whats-the-relationship-between-drive-and-robotstateestimator)
		- [Does their OI allow driver to enter auto-like modes during teleop?](#does-their-oi-allow-driver-to-enter-auto-like-modes-during-teleop)
- [Operator Interface](#operator-interface)
	- [How do UI events trigger robot actions, are network tables involved?](#how-do-ui-events-trigger-robot-actions-are-network-tables-involved)
	- [If a subsystems wants access to a joystick, how is this obtained?](#if-a-subsystems-wants-access-to-a-joystick-how-is-this-obtained)
	- [At what point is field position detected for autonomous behavior?](#at-what-point-is-field-position-detected-for-autonomous-behavior)
	- [Where does joystick remapping occur? what remapping functions are applied?](#where-does-joystick-remapping-occur-what-remapping-functions-are-applied)
	- [Which ControlBoardInterface is actually employed?](#which-controlboardinterface-is-actually-employed)
	- [Who owns the responsibility for inverting the sense of joystick directions?](#who-owns-the-responsibility-for-inverting-the-sense-of-joystick-directions)
	- [How/when/where is the smart dashboard updated?](#howwhenwhere-is-the-smart-dashboard-updated)
	- [Is there a single location/database for all button and joystick identifiers?](#is-there-a-single-locationdatabase-for-all-button-and-joystick-identifiers)
- [RobotStateEstimator](#robotstateestimator)
	- [What is the job of `RobotStateEstimator`?](#what-is-the-job-of-robotstateestimator)
- [RobotState](#robotstate)
	- [What is the purpose of `RobotState`?](#what-is-the-purpose-of-robotstate)
	- [What are its frames of interest?](#what-are-its-frames-of-interest)
	- [What is a `kinematic chain`?](#what-is-a-kinematic-chain)
	- [How do we convert coordinates between frames of interest?](#how-do-we-convert-coordinates-between-frames-of-interest)
	- [How is it we can look up the position of the robot at any point in the past?](#how-is-it-we-can-look-up-the-position-of-the-robot-at-any-point-in-the-past)
	- [Why do we care about _past_ robot position and orientation?](#why-do-we-care-about-past-robot-position-and-orientation)
- [Vision](#vision)
	- [What camera did 254 use? How were vision targets delivered to the robot](#what-camera-did-254-use-how-were-vision-targets-delivered-to-the-robot)
	- [What threads are involved in delivering vision targets?](#what-threads-are-involved-in-delivering-vision-targets)
	- [What is the processing required to act upon vision target acquisition?](#what-is-the-processing-required-to-act-upon-vision-target-acquisition)
	- [Why is `TargetInfo`'s X coordinate always zero?](#why-is-targetinfos-x-coordinate-always-zero)
- [Paths](#paths)
	- [what is the output format of the cheesy_path webapp?](#what-is-the-output-format-of-the-cheesypath-webapp)
		- [How are these paths brought into the robot code?](#how-are-these-paths-brought-into-the-robot-code)
	- [How are these paths used to affect the drive trajectory?](#how-are-these-paths-used-to-affect-the-drive-trajectory)
	- [What is PathFollower and how does it work?](#what-is-pathfollower-and-how-does-it-work)
	- [What is an AdaptivePurePursuitController?](#what-is-an-adaptivepurepursuitcontroller)
	- [What is a Path?](#what-is-a-path)
	- [What is a PathSegment?](#what-is-a-pathsegment)
	- [What is a MotionProfile?](#what-is-a-motionprofile)
	- [What is a MotionSegment?](#what-is-a-motionsegment)
	- [What is a MotionState?](#what-is-a-motionstate)
- [References](#references)

<!-- /TOC -->

### Basic execution model, bootstrapping, debugging

#### how does our code get built, downloaded and launched?
> our build system relies on `ant` which describes the build process
> with a series of .xml and .properties files.  Ultimately it
> creates a file named `FRCUserProgram.jar`. This file and associated
> runtime libraries are copied down to the robot (to /home/lvuser and
> to /usr/local/frc/lib).
> Another file, `robotCommand` (or `robotDebugCommand`) is also copied. This
> file is **NOT** a shell script but rather a command file that is parsed by
> `/usr/local/frc/bin/frcRunRobot.sh` and ultimately passed to
> `/sbin/start-stop-daemon` whose job is to 'watch over'
> the robot process - to restart it if it crashes, to start it on robot
> reboot, etc.  When we deploy a robot-program that crashes, the start-stop-daemon
> can produce an annoying and confusing flurry of program restarts.  The only
> way to stop the flurry is either to kill the process group or to deploy a
> stable robot program.  The output of the robot program is also routed over the
> network via this startup machinery.

#### where are crashlogs found?  How did they get there?
> /home/lvuser/crash_tracking.txt is created by Logger (formerly CrashTracker)
> There's also a log of program output produced by the bootstrapping mechanism
> located here: `/var/local/natinst/log/FRC_UserProgram.log`. It may be
> helpful to create a symbolic link to this file in the lvuser home.

#### where is java main? When does our code get control?
> `main()` is the primary/default entrypoint for any java program. In our
> case, it is located in WPILib's RobotBase class.  Our primary entrypoint
> is the constructor of our Robot class and this occurs as a side-effect
> of `RobotBase::main()`. Our robot implements the `IterativeRobot` subclass
> and, as such, does not implement the primary control loop. Rather the
> IterativeRobot must implement `Init` and `Periodic` variants for each of these
> robot states:  autonomous, teleop, test, and disabled. During each
> iteration of the primary control loop, `robotPeriodic()` is invoked and
> both the SmartDashboard and LiveWindow are updated.  Note that most
> robot initialization code should happen during `Robot::robotInit` and
> *NOT* the constructor. Similarly, communication between DriverStation
> and robot should take place during autonomousInit(), not robotInit(), since
> robotInit runs during powerup which is way before the match starts.

#### what is the `Looper`.  When is it created? When is it started/stopped?
> The Looper class is instantiated by our robot in `robotInit()` under the
> name `mEnabledLooper`.  During robotInit, Subsystems are afforded the opportunity
> to register code for repeated execution (aka _looping_) by implementing the
> `Loop` interface which is comprised of these methods: `onStart()`,
> `onLoop()`, `onStop()`. The looper uses the WPI class: `Notifier` which
> is described as providing a notification callback for timer events.
> Notifier creates a separate execution thread and so the callbacks into
> subsystem loops is running in a thread different from robot methods.
> Thus we must _synchronize_ state change requests made by the main thread
> (or the AutoMode thread) with the looper thread. Since the purpose of
> the looper thread is to distribute time cycles to each subsystem, any
> calls to Thread.sleep() by implementors of Loop would have the effect of
> starving other subsystem Loops.  _This would be bad_. The looper is
> started in `autonomousInit()` and again in `teleopInit()` (twice is fine).
> the looper is stopped during disabledInit() and this implies that the
> loopers aren't running during testPeriodic unless autoInit or teleopInit
> has been called without disabledInit().  The looper is configured via
> Constant.kLooperDt which is set to .005 (1/200 sec)  The measured
> duration of a single loop (summing the onLoop() calls for all Loops)
> is reported to the smart dashboard as `looper_dt`.

#### what does `registerEnabledLoops()` do?  Who calls it?
> `Robot:robotInit()` invokes `SubsystemManager:registerEnabledLoops`
> which, in turn, invokes this method on each subsystem. This allows
> each subsystem to register code (implemented as a `Loop` subclass) to execute
> in the Looper thread.

### Threads
#### what is a `Thread`?
> A thread is an independent 'stream' of code that can execute simultaneously
> with other threads within the same java _process_. A robot program is
> typically populated with 2-5 threads because sampling hardware and
> network states are operations that require a substantial and unpredictable
> amount of waiting.  While one thread is waiting, others can be performing
> useful work. The biggest challenge in writing _multithreaded_ applications
> involves synchronizing the activities of independent threads. This is
> a challenge because multiple threads might be reading and writing to the
> same value or device.  Because the timing of these reads and writes is
> often not predictable, the results of poorly written multithreaded apps
> can be unpredictable, buggy and even crash-prone.

#### what is a `Runnable`,  `CrashTrackingRunnable`?
> A `Runnable` is a standard java interface that requires the implementation
> of a single method: `run()`. A Runnable is required when creating
> new threads. Upon creation of a new execution thread, the `run()`
> method is invoked. `CrashTrackingRunnable` is an implementation
> of `Runnable` that implements `run()` as a call to `runCrashTracked()`
> nested within a try/catch block. Subclasses of `CrashTrackingRunnable`
> must implement `runCrashTracked()` as opposed to `run()`.

* what does a `synchronized` method imply?
> synchronized implies that a method is intended to be called by multiple threads.
> The 'java runtime' guarantees that a synchronized method will only be running
> in one thread at a time. This is a simple method for changing the state
> of a object that is shared across multiple threads. The `synchronized`
> keyword is used in other ways but generally means predictable code execution
> in multithreaded environments.

#### what guarantees does Thread.sleep() offer?  How many invocations of this
  method are there in the entire codebase?
>  Thread.sleep() is used to _pause_ the execution of the current thread for
> a designated time interval.  It does not guarantee that the time duration
> will always be respected as it may throw an InterruptedException under a
> variety of conditions.  Furthermore, even when it returns normally, the
> accuracy of the interval varies from system to system. Thus, it should
> be used to give cycles to other threads in 'slow' polling scenarios, but
> it should not be used to perform time-sensitive operations. There are
> several invocations in our codebase. The most important are found in
> `AutoModeBase` and `CANProbe`.

### Commands, Scheduler
#### how is the autonomous mode selected?  How does the selected mode receive
  cycles (updates)?
> Our class `AutoModeSelector` presents a list of choices to the Driverstation
> via the network tables.  Note that we don't employ WPI's SendableChooser
> because it doesn't support sufficient control over GUI presentation.
> During `autonomousInit()`, we invoke `getSelectedAutoMode()` which
> returns a subclass of `AutoModeBase`. This is passed to an instance
> of `AutoModeExecuter`, which creates a thread responsible for scheduling
> the sequence of actions required by the selected mode.  AutoModeBase subclasses
> must implement `routine()` which is called (once) from `AutoModeBase::run()`.
> AutoModeBase implements the method `runAction(Action)` which is expected to be
> invoked by AutoModeBase subclasses. Typically, an AutoModeBase is comprised
> of a collection of Actions that can be run serially (`SeriesAction`) and/or
> in parallel (`ParallelAction`);

#### how does the autonomous mode control subsystems? What’s the difference
  between an auto mode and an auto action?
> Autonomous modes don't directly control subsystems, instead they schedule
> actions which interact directly with subsystems. Actions are like commands,
> and modes are like command groups in traditional WPILib. Remember: the
> Looper is running Subsystem methods in a separate thread.  This is the
> means by which, for example, a motor value is updated with high regularity.

#### what is the frequency of the scheduler loops, where is this embodied?
> We have different scheduler loops for autonomous and for teleop. The
> autonomous loop frequency is governed by `AutoModeBase::m_update_rate` which
> is set to 1/50 seconds and used as the parameter to Thread.sleep().
> During auto and teleop, the Looper is responsible for periodic subsystem updates.
> As described above, the default period is controlled via Constants.kLooperDt (.005s).
> The autoPeriodic and teleopPeriodic frequencies are governed by the
> IterativeRobot which invokes DriverStation.waitForData(0) which waits
> for data delivered periodically from the driverstation.  WPI docs
> [here](https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599697-choosing-a-base-class)
> state that the periodic routines run approximately every 20 milliseconds
> (50 times per second).

#### how many threads are running at the same time in auto and tele?
> Minimally, there are two threads running in each of these modes.
> The first thread is the one that runs the robot methods (autoPeriodic,
> telePeriodic).  The second thread is the Looper. During auto AutoModeExecuter
> is also likely to be running. Additional threads may be running by robot code
> through RobotSafety, PIDControl, Notifier, and other interfaces.  We anticipate
> running a separate thread to receive vision target updates. Here's is a
> typical dump that shows extra looper threads being shutdown
>
```
 NOTICE  disabled init
 Stopping loops
 Stopping com.spartronics4915.frc2018.subsystems.Superstructure$1@75cab9
 Stopping com.spartronics4915.frc2018.subsystems.ConnectionMonitor$1@f7fe8e
 Stopping com.spartronics4915.frc2018.subsystems.LED$1@179caec
 Stopping com.spartronics4915.frc2018.subsystems.Testbed$1@1117f44
 Stopping com.spartronics4915.frc2018.loops.VisionProcessor@1d3411d
 Stopping com.spartronics4915.frc2018.loops.RobotStateEstimator@71cca7
```

#### what is a “wantedState” as specified in Robot::teleopPeriodic?
> a wanted state is conveyed by autonomous or teleop to a subsystem.
> The subsystem is expected to respond to this by transitioning from its
> current, or measured, state to the wantedState.  In the case of the
> Superstructure subsystem, a single wantedState might be translated to
> individual wantedStates for a subset of subsystems. It is not guaranteed
> that the machine actually transitions into this state, but it does make
> it easy to see what state the robot is endeavoring to achieve.
> This is in contrast to classic WPILib, where the it is extremely difficult
> to determine the robot's state at any given time. Again, multiple states are
> coordinated by Superstructure, and usually the Robot sets the wanted state
> of superstructure. Less frequently, Robot sets wanted state of a subsystem
> directly.

### Subsystem
#### how many subsystems are there? Do they share a base class?
> There are 10 subsystems in the original 2017 code. The `Subsystem` class
> is the abstract definition of the required behavior and methods of a subsystem.
> It also offers shared behavior for logging messages.

#### is there a single database of motor identifiers, gpio pins, etc?
> Yes, the file `Constants.java` replaces `RobotMap.java` from prior years.
> The file is divided into hardware constants and software constants.
> Hardware constants are organized by _interface_, not by subsystem.
> This means that all the TalonSRX device ids reside within the CAN section
> of the hardware section. Similarly all of the consumers of AnalogInput
> pins are adjacent to each other.

#### can subsystems modify the state of other subsystems?
> while it is theoretically possible for any subsystem to obtain an instance
> of another subsystem, it is generally frowned upon.  There is a special
> subsystem, `Superstructure`, whose primary role is to coordinate
> cross-subsystem behavior. This means that the Robot generally communicates
> requirements through Superstructure and not to individual subsystems.

#### what is the purpose of `SubsystemManager`?
> it is a simple 'broadcast mechanism' to distribute standard subsystem method
> calls across all registered subsystems.  Note that it isn't a Subsystem, but
> rather merely a collector of all subsystems.

#### LED
The LED subsystem is the simplest, easiest to understand.  Probably the
best place to start if you're trying to understand Subsystem design.
##### when/how does the LED subsystem determine what blinking pattern to emit?
   How does it make a blinking pattern?
> LED is given a wanted state when `setWantedState()` is invoked. The
> next time it loops it tries to synchronize the wanted state with the current
> state. It blinks and does things based on the state and the time spent in
> that state. To get blinking behavior it performs integer division with
> timeInState as the divisor and mBlinkDuration / 2 as the dividend. If
> the quotient of the operation is even then the light should be on, otherwise,
> it should be off.

#### Superstructure
##### how is this subsystem different from others?
> The SuperStructure subsystem is the coordinator of those subsystems that
> must be controlled in concert.  Potential Superstructure states are described
> by coordinated per-subsystem states (arm is down and hand is ready to grab).
> While there are exceptions to the rule, generally the Superstructure's only
> job is coordination and not direct control of actuators.

#### Drive
##### How many drive motors? What is a follower motor?
> There are 4 motors in this standard drive train.  Each Robot side has two
> motors driving a gear box and so each side's motors must therefore output
> the same RPMs to its drive shaft.  Rather than send two independent signals
> to two motors on the same side, we use TalonSRX's _follower_ control mode. A
> follower motor is initialized with the id of its _master_ and while running
> in the follower control mode, it will mimic all of its master's actions. This
> means that in order to control four motors we only need to emit two control
> signals, one to each master motor controller.

##### How are encoders handled?  How are encoder ticks converted to user-units?
> There are a number of Drive constants in Constants.java. There are
> also a number of Drive methods that convert between the various Speed
> and distance measures (rpm, inches/sec, etc).

##### What is CheesyDrive?  How does it operate?
> from the code:
```
 /**
 * "Cheesy Drive" simply means that the "turning" stick controls the curvature
 * of the robot's path rather than its rate of heading change. This helps make
 * the robot more controllable at high speeds. Also handles the robot's quick
 * turn functionality - "quick turn" overrides constant-curvature turning for
 * turn-in-place maneuvers.
 */
```

##### In teleop, are we operating in setVelocity or in PercentOutput mode?
> it runs open-loop (PercentOutput mode).

##### When do they use positionControlMode?
> AIM_TO_GOAL, DRIVE_TOWARDS_GOAL, and TURN_TO_HEADING (in the last case to
> make sure that the encoders are really where they say they are, I think.
> They do this with they gyro in other cases too, and I'm not completely
> sure I understand the rationale. (TBD)

##### What units do they use in velocity control mode?
> inches per second

##### How many internal states does the Drive subsystem have? What do they do?
> Drive currently has seven states:
> * **OPEN_LOOP** - basic manual drive mode in which Percent Output is the
>   control signal. Encoder values are not consulted in this mode, only driver
>   control-board (ie joystick) values affect it.
> * **VELOCITY_SETPOINT** - this closed-loop control mode can be employed in
>   teleop, where the driver control-board specifies velocities for left and
>   right. In autonomous settings, the target velocities must be computed
>   according to some strategy.  To support multiple velocity control strategies
>   we utilize the notion of internal state to characterize each combination.
> * **PATH_FOLLOWING** - employs close-loop velocity control to follow a
>   requested path.  Based on the curvature and target path velocity
>   this mode computes the target velocities for each motor.
> * **AIM_TO_GOAL** - After identifying a goal, we transition to the
>   TURN_TO_HEADING state.
> * **TURN_TO_HEADING** - employs closed-loop position control to cause the robot
>   to turn in place to face the specified goal. This approach only uses encoders
>   to measure rotation, and not the gyro. The robot is said to have achieved
>   the target heading when the heading *and* velocity are within a specified
>   tolerance.  Note that in this mode we use the Kinematics module to convert
>   a rotation angle to delta positions for left and right wheels. These are
>   added to the current encoder positions to obtain a new setpoint.  Also
>   note that the target is specified in field position, so we obtain the
>   robot orientation in field position in order to determine a heading error.
> * **DRIVE_TOWARDS_GOAL_COARSE_ALIGN** - employs closed-loop position control
>   to first perform a course direction-alignment which, when reached,
>   state-transitions to DRIVE_TOWARDS_GOAL_APPROACH.
> * **DRIVE_TOWARDS_GOAL_APPROACH** - employs closed-loop position control to
>   cause the robot to move straight forward or reverse in order to achieve
>   an optimal shooting distance from the goal.  Upon reaching the target
>   distance we state-transition the drive to AIM_TO_GOAL state.

##### Do any internal states lead automatically to other internal states?
> Yes, in fact, most of them do and this shows how high-level state requests
> can result in multiple lower-level state transitions.

##### Are MotorSafety settings in play?  If so, how do we prevent
	“Output not updated enough...” messages?
> Because CANTalon implements WPI's MotorSafety interface, they are in
> play unless we specifically disable them via `setSafetyEnabled(false)`.
> To prevent these messages we must invoke the motor's `.set()` more
> frequently than the safety interval `MotorSafety:DEFAULT_SAFETY_EXPIRATION`.
> Due to the fact that our Looper is running Loops at a regular interval,
> we must ensure that a Subsystem with motors must invoke `set` on each
> call to `onLoop()`. The `stop()` method of the subsystem should either
> disable safety or repeatedly invoke stop.

##### What’s the relationship between Drive and RobotStateEstimator?
> RobotStateEstimator gets a bunch of info from Drive and feeds it into
> Kinematics, whose output it finally feeds into RobotState to be stored
> as a transform that represents an estimate of the Robot's position
> and orientation on the field.

##### Does their OI allow driver to enter auto-like modes during teleop?
> Yes, it does allow them to enter autonomous like modes.  See
> `Drive.setWantAimToGoal()` and the like.

### Operator Interface
#### How do UI events trigger robot actions, are network tables involved?
> Dashboard UI events trigger robot behavior using NetworkTables. Currently,
> this only happens via our `AutoModeSelector`, which is read in Robot.java
> when auto begins.

#### If a subsystems wants access to a joystick, how is this obtained?
> Driverstation Joysticks and Buttons are delivered to the robot via the
> standard WPILib conduits.  `Robot:teleopPeriodic` is where we poll buttons
> and joysticks and invoke the appropriate subsystem methods.

#### At what point is field position detected for autonomous behavior?
> Field position is never 'detected', but driverstation 'location' is.  
> During `Robot::autonomousInit()` we can employ the `DriverStation`
> methods: `getAlliance()` and  `getGameSpecificMessage()` to obtain
> per-match data.  Note that this information isn't available during
> robot construction.

#### Where does joystick remapping occur? what remapping functions are applied?
> There is an interface called `ControlBoardInterface` that allows for
> easy switching between different control schemes (as long as they satisfy
> the interface), but remapping is done with `CheesyDriveHelper` which accepts
> some joystick information from a `ControlBoardInterface` and does a lot of math
> that isn't well motivated in the code. The response seems pretty nice though.

#### Which ControlBoardInterface is actually employed?
>  In the unmodified code a class called `ControlBoard` that implements
> `ControlBoardInterface` is used.  A new method has been added that also
> satisfies the interface to get the right mappings for the Xbox controller.

#### Who owns the responsibility for inverting the sense of joystick directions?
> Implementors of `ControlBoardInterface` are responsible for mapping the
> knobs and buttons into a canonical form. Directions and mappings should
> remain behind the ControlBoard abstraction.

#### How/when/where is the smart dashboard updated?
> The most significant SmartDashboard updates happen when `Robot.allPeriodic()`
> is called. There we invoke `SubsystemManager.outputToSmartDashboard()`,
> which in turn calls that same method its list of all subsystems.
> Note that the Subsystem interface _requires_ all subsystems to implement
> this method.

#### Is there a single location/database for all button and joystick identifiers?
> Yes - its the ControlBoardInterface.  Note that each year this interface
> must be changed to reflect control board abstractions (eg: squeeze grabber
> button).  Clients of the interface never rely on specify button mappings
> so it's natural for all the choices to be localized to the code that
> implements the entire interface.

### RobotStateEstimator

#### What is the job of `RobotStateEstimator`?
> The job of RobotStateEstimator is to update the RobotState with a computation
> of distance traveled by each side of the robot.
```
final Twist2d odometryV = mRobotState_.generateOdometryFromSensors()
        sensorLeft - lastSensorLeft, sensorRight - lastSensorRight, gyroAngle);
final Twist2d predictedV = Kinematics.forwardKinematics(mDrive.getLeftVelocityInchesPerSec(),
                                        mDrive.getRightVelocityInchesPerSec());
mRobotState_.addObservations(timestamp, odometryV, predictedV);
```
> Note that `mRobotState.generateOdometryFromSensors()` converts deltaLeft,
> deltaRight and gyroAngle, into a `Twist2d` representing the current velocity
> vector.

### RobotState

#### What is the purpose of `RobotState`?
> RobotState keeps track of the poses of various coordinate frames throughout
> the match. A coordinate frame is simply a point and direction in space that
> defines an (x,y) coordinate system. Transforms (or poses) keep track of the
> spatial relationship between different frames.

#### What are its frames of interest?
> * `Field frame`: origin is where the robot is turned on. By convention,
>   the field represents the xy plane with the z direction pointing up.
>   The x axis is the long axis of the field.
> * `Vehicle frame`: origin is the center of the robot wheelbase, facing
>    forwards. This frame is relative to the field frame. A zero rotation and
>    a zero translation would place the center of the robot at the "bottom left",
>    of the field viewed from the top.
> * `Camera frame`: origin is the center of the camera imager relative
>   to the robot frame. A zero camera rotation places the viewing
>   direction along increasing x.
> * `Goal frame`: origin is the center of the target (note that orientation in
>   this frame is arbitrary). Also note that there can be multiple target frames.
>   Each goal, then, is easily represented in field coordinates as well.

#### What is a `kinematic chain`?
> A linked series of transformations from one frame to another is known as
> a kinematic chain. If we're given a position in the camera frame, we can
> compute, for example, its coordinates in the field frame by performing
> coordinate conversion in the _forward_ > direction.  If we're given a
> position in the field frame, we convert that to the robot frame by
> performing coordinate conversion in the _inverse_ direction. Ours is
> a kinematic chain with 4 frames, and so there are 3 transforms of interest:
> 1. Field-to-vehicle: This is tracked over time by integrating encoder and
> gyro measurements. It will inevitably drift, but is usually accurate over
> short time periods.
> 2. Vehicle-to-camera: This is a constant unless the camera is mounted on a
>   turret.
> 3. Camera-to-goal: This is a pure translation, and is measured by the vision
>   system.

#### How do we convert coordinates between frames of interest?
> `RigidTransform2d` is the class responsible for transforming points
> from one coordinate frame to another. It is composed of a Translation2d
> and a Rotation2d.  Rotation2d is represented as an angle (which is decomposed
> into cos_theta and sin_theta, presumably for performance).  Another geometric
> type is Twist2d which is used to represent movement along an arc of constant
> curvature at a constant velocity. We can apply a Twist2d to a RigidTransform2d
> to produce a new RigidTransform2d and this represents a form of incremental
> (turning) motion.

#### How is it we can look up the position of the robot at any point in the past?
> `RobotState` maintains a sorted list of robot positions:
```
private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> mFieldToVehicle;
```
> where the key to this map is the timestamp associated with the robot's
> position on the field (again, relative to its starting location and
> orientation). Each timeslice, RobotStateEstimator issues a call to
> `addObservation` to append a new sample to this map.

#### Why do we care about _past_ robot position and orientation?
> Because of fundamental latencies in our vision sampling and processing
> targets are actually found relative to a past position.  In order to
> understand the target location in terms of current robot position,
> we must compute the target position in field coordinates, then convert
> them to the _current_ robot frame.

### Vision
#### What camera did 254 use? How were vision targets delivered to the robot
  process?
> 254's implementation depended upon an on-board android phone running a custom
> vision app and communicating state via a internet-over-usb connection hosted
> on the robot side by a port of adb (android debugger). They had a separate
> thread that launched and kept the adb connection alive.  They also had a separate
> vision control thread to receive data from adb and update robot target state.
> We have the option of replacing the adb communication path with networktables.
> Now, we can either employ their VisionProcessor thread to sample the
> networktable values, or we can register a notification callback in the main
> thread.

#### What threads are involved in delivering vision targets?
> `VisionProcessor` implements Loop and VisionUpdateReceiver.  Its job is to
> to deliver VisionUpdates (received via synchronized gotUpdate())
> to RobotState.  `RobotStateEstimator` is also running in a separate thread
> and computes/delivers updates to the RobotState based on the `Drive` sensors.
> Note that VisionProcessor sends data directly to RobotState in a manner
> analogous to the RobotStateEstimator.  In that sense, the VisionProcessor
> can be thought of as a `GoalStateEstimator`.

#### What is the processing required to act upon vision target acquisition?
> Remember that the goal is captured in the coordinate frame of the camera
> and must be converted through the coordinate frame of the robot
> _at the time of capture_ (ie in the past) to the field coordinate frame,
> then back to the _current_ robot coordinate frame in order to inform robot
> motion planning. In addition, we must prevent against instability
> in our tracking algorithm.  In a field with multiple cubes, a
> random selection between multiple contenders would make for a
> random robot dance and reduce the change of a cube capture
> significantly. RobotState has a `GoalTracker` class to help
> prioritize incoming targets according to a series of heuristics
> embodied by the class `TrackReportComparitor`. If we wish to
> support multiple live targets, then we need a list of active
> targets.  An indivdual target is embodied via `GoalTrack` Which
> maintains a list of observed positions and their associated
> timestamps. In our cube-tracking implementation we might simplify
> the problem by accepting only a single cube and, moreover adopting
> a single nearest-one-wins heuristic. In that setting it might be
> best to implement the target selection and tracking code on the
> vision processor and not in robot code.

#### Why is `TargetInfo`'s X coordinate always zero?
> Here's their comment justifying the reassigment of the camera's x to the
> target Y and the camera's Y to target Z.
> Also of note is the sign conversion.
```
// Convert to a homogeneous 3d vector with x = 1
double y = -(target.centroidX - kCenterCol) / getFocalLengthPixels();
double z = (target.centroidY - kCenterRow) / getFocalLengthPixels();
```
> Presumably, this is done in order to convert to a robot coordinate
> system?  As for focal length, it can be optained as follows:
```
double focal_length_pix = (size.width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180);
```

### Paths
#### what is the output format of the cheesy_path webapp?  	
> The cheesypath webapp outputs java code suitable for inclusion in a robot
> project.  The code takes the form of a custom class that implements PathContainer,
> with an `ArrayList<WayPoint>` representing the results of user interaction.

##### How are these paths brought into the robot code?
> The results of cheesy_path are included in robot code by adding the
> custom class java file directly to the repository.  By convention, custom
> PathContainers are placed in a game-specific area: `frc2018/paths`.  
> Next an instance of this class can be created as a component of an
> autonomous mode.  The file, `frc2018/auto/modes/TestPathMode` is an
> example that instantiates the class TestPath and instantiates an instance
> of `DrivePathAction`, parameterized by the TestPath instance.

#### How are these paths used to affect the drive trajectory?
> `DrivePathAction` starts off the path follower by issuing
> `mDrive.setWantDrivePath(mPath, mPathContainer.isReversed())`.
> Drive then configures its motors into VelocityControlMode, constructs
> an instance of PathFollower and enters the control state, `PATH_FOLLOWING`.
> Now the drive's looper periodically invokes the drive's `updatePathFollower()`.
> Here, we consult `RobotState` for an estimate of our current field
> position. This _robot pose_ is passed to the PathFollower's `update` method
> whose job it is to produce a `Twist2d` that represents target velocities
> required to update the left and right velocity targets for the motors.
> Note that the single value for Twist2d represents the path for the robot center.
> This must converted to left and right robot velocities via
> `Kinematics.inverseKinematics()` which _knows_ the geometry of the robot and
> implements the conversion on the assumption of differential drive kinematics.

#### What is PathFollower and how does it work?
> PathFollow is the centeral class that implements path following. Drive
> creates an instance of this class when it enters PATH_FOLLOWING control mode.
> Instantiation parameters are numerous and can be found in `Constants.java`.
> Important path following constants include max values for the look-ahead and
> speed as well as PID values for the closed loop controller.
> Here's the introduction comment from the code:
```
 /**
 * A PathFollower follows a predefined path using a combination of feedforward
 * and feedback control. It uses an AdaptivePurePursuitController to choose a
 * reference pose and generate a steering command (curvature), and then a
 * ProfileFollower to generate a profile (displacement and velocity) command.
 */
```
> PathFollower has a number of important member variables:
> * AdaptivePurePursuitController mSteeringController;
> * ProfileFollower mVelocityController;
>
> The heart of PathFollower is its `update()` method. Its parameter are:
> * t: the current timestamp
> * pose:  the current robot pose (a RigidTransform2d)
> * distance: the distance along the path the robot has already traveled
> * velocity: the current speed of the robot (directionless)

#### What is an AdaptivePurePursuitController?
> from the code comments:
```
 /**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4
 * .pdf
 *
 * Basically, we find a spot on the path we'd like to follow and calculate the
 * arc necessary to make us land on that spot. The target spot is a specified
 * distance ahead of us, and we look further ahead the greater our tracking error.
 * We also return the maximum speed we'd like to be going when we reach the
 * target spot.
 */
```

#### What is a Path?
> A Path is a List<PathSegment>.  The current segment represents a part of
> the entire path that we are currently following. When we arrive at the end
> of the current segment, it is removed from the list and we adopt a new
> current segment.  The current segment is always at index 0 of our list and
> we know we've reached our target when the PathSegment list is empty. Presumably,
> a Path has geometric continuity at segment boundaries. This constraint must
> be enforced by the Path authoring system.

#### What is a PathSegment?
> A PathSegment is a component of a larger path that represents either a
> linear or circular arc with target speeds at each endpoint. In addition
> to the geometric descriptors, a PathSegment also has-a `MotionProfile`
> named `speedController`. Central to the functionality of PathSegment are
> these methods:
> * `getClosestPoint()`
> * `getPointByDistance()`
> * `getSpeedByClosestPoint()`
> * `getSpeedByDistance()`

#### What is a MotionProfile?
> from code comments:
```
 /**
 * A motion profile specifies a 1D time-parameterized trajectory. The trajectory
 * is composed of successively coincident MotionSegments from which the desired
 * state of motion at any given distance or time can be calculated.
 */
```

#### What is a MotionSegment?
> from code comments:
```
 /**
 * A MotionSegment is a movement from a start MotionState to an end MotionState
 * with a constant acceleration.
 */
```

#### What is a MotionState?
> from code comments:
```
 /**
 * A MotionState is a completely specified state of 1D motion through time.
 */
```
> member variables:
> * double t;
> * double pos;
> * double vel;
> * double acc;

### References
[Coding Notes](DesignNotes.md) |
[Porting Notes](PortingNotes.md) |
[Cheesy Notes](CheesyNotes.md) |
[254 Github](https://github.com/Team254)
