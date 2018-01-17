## Learning Spartronics-254Base

### Basic execution model, bootstrapping, debugging

* how does our code get built, downloaded and launched?
> our build system relies on `ant` which describes the build process
> with a series of .xml and .properties files.  Ultimately it
> creates a file named `FRCUserProgram.jar`. This file and associated
> runtime libraries are copied down to the robot, then executed by
> the shell script, `/home/lvuser/robotCommand`.  This script is executed by
> other scripts on the system whose job is to ensure the robot program
> is started even after reboots. The output of the program is also
> routed over the network via this startup machinery.

* where are crashlogs found?  How did they get there?
> /home/lvuser/crash_tracking.txt by CrashTracker.  There's also
> a log of program output produced by the robotCommand launcher
> (TODO: where is that log?).

* where is java main? When does our code get control?
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

* what is the `Looper`.  When is it created? When is it started/stopped?
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

* what does `registerEnabledLoops()` do?  Who calls it?
> `Robot:robotInit()` invokes `SubsystemManager:registerEnabledLoops`
> which, in turn, invokes this method on each subsystem. This allows
> each subsystem to register code (implemented as a `Loop` subclass) to execute
> in the Looper thread.

### Threads
* what is a `Thread`?
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

* what is a `Runnable`,  `CrashTrackingRunnable`?
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

* what guarantees does Thread.sleep() offer?  How many invocations of this
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
* how is the autonomous mode selected?  How does the selected mode receive
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

* how does the autonomous mode control subsystems? What’s the difference
  between an auto mode and an auto action?
> Autonomous modes don't directly control subsystems, instead they schedule
> actions which interact directly with subsystems. Actions are like commands,
> and modes are like command groups in traditional WPILib. Remember: the
> Looper is running Subsystem methods in a separate thread.  This is the
> means by which, for example, a motor value is updated with high regularity.

* what is the frequency of the scheduler loops, where is this embodied?
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

* how many threads are running at the same time in auto and tele?
> Minimally, there are two threads running in each of these modes.
> The first thread is the one that runs the robot methods (autoPeriodic,
> telePeriodic).  The second thread is the Looper. During auto AutoModeExecuter
> is also likely to be running. Additional threads may be running by robot code
> through RobotSafety, PIDControl, Notifier, and other interfaces.  We anticipate
> running a separate thread to receive vision target updates.

* what is a “wantedState” as specified in Robot::teleopPeriodic? What is the
  difference between WantedState and SystemState? How are the
   multiple states of each subsystem coordinated?
> a wanted state is conveyed by autonomous or teleop to a subsystem.
> The subsystem is expected to respond to this by transitioning from its
> current, or measured, state to the wantedState.  In the case of the
> Superstructure subsystem, a single wantedState might be translated to
> individual wantedStates for a subset of subsystems. It is not guaranteed
> that the machine actually transitions into this state, but it does make
> it easy to see what state the robot is endeavoring to achieve.
> This is in contrast to classic WPILib, where the it is extremely difficult
> to determine the robot's state at any given time. Again, multiple states are
> coordinated by Superstructure, and most of the rob code sets the wanted state
> of superstructure. Less frequently, robot code sets wanted state of a subsystem
> directly.

### Subsystem
* how many subsystems are there? Is there a base class that establishes
  all the interface conventions for subsystems?
> There are 10 subsystems in the original 2017 code. The `Subsystem` class
> is the abstract definition of the required behavior and methods of a subsystem.
> It also offers shared behavior for logging messages.

* is there a single database of motor identifiers, gpio pins, etc?
> Yes, the file `Constants.java` replaces `RobotMap.java` from prior years.
> The file is divided into hardware constants and software constants.
> Hardware constants are organized by _interface_, not by subsystem.
> This means that all the TalonSRX device ids reside within the CAN section
> of the hardware section. Similarly all of the consumers of AnalogInput
> pins are adjacent to each other.

* can subsystems modify the state of other subsystems?
> while it is theoretically possible for any subsystem to obtain an instance
> of another subsystem, it is generally frowned upon.  There is a special
> subsystem, `Superstructure`, whose primary role is to coordinate
> cross-subsystem behavior. This means that the Robot generally communicates
> requirements through Superstructure and not to individual subsystems.

* what is the purpose of `SubsystemManager`?
> it is a simple 'broadcast mechanism' to distribute standard subsystem method
> calls across all registered subsystems.  Note that it isn't a Subsystem, but
> rather merely a collector of all subsystems.

#### LED
The LED subsystem is the simplest, easiest to understand.  Probably the
best place to start if you're trying to understand Subsystem design.
* when/how does the LED subsystem determine what blinking pattern to emit?
   How does it make a blinking pattern?
> LED is given a wanted state when `setWantedState()` is invoked. The
> next time it loops it tries to synchronize the wanted state with the current
> state. It blinks and does things based on the state and the time spent in
> that state. To get blinking behavior it performs integer division with
> timeInState as the divisor and mBlinkDuration / 2 as the dividend. If
> the quotient of the operation is even then the light should be on, otherwise,
> it should be off.

#### Superstructure
* how is this subsystem different from others?
> The SuperStructure subsystem is the coordinator of those subsystems that
> must be controlled in concert.  Potential Superstructure states are described
> by coordinated per-subsystem states (arm is down and hand is ready to grab).
> While there are exceptions to the rule, generally the Superstructure's only
> job is coordination and not direct control of actuators.

#### Drive
* how many internal states does the Drive subsystem have?
> The 254-2017 Drive subsystem had 7 internal states.

* how many actuators, how many sensors?
> The 254-2017 Drive subsystem had 5 actuators and 3 sensors.

* how many follower-mode motors are there?
> The 254-2017 Drive subsystem had 2 masters, 2 followers

* are MotorSafety settings in play?  If so, how do we prevent
	“Output not updated enough...” messages?
> Because CANTalon implements WPI's MotorSafety interface, they are in
> play unless we specifically disable them via `setSafetyEnabled(false)`.
> To prevent these messages we must invoke the motor's `.set()` more
> frequently than the safety interval `MotorSafety:DEFAULT_SAFETY_EXPIRATION`.
> Due to the fact that our Looper is running Loops at a regular interval,
> we must ensure that a Subsystem with motors must invoke `set` on each
> call to `onLoop()`. The `stop()` method of the subsystem should either
> disable safety or repeatedly invoke stop.

* how are encoders handled?  how are encoder ticks converted to user-units?
> There are a number of Drive constants in Constants.java. There are
> also a number of Drive methods that convert between the various Speed
> and distance measures (rpm, inches/sec, etc).

* what’s the relationship between Drive and RobotStateEstimator?
> RobotStateEstimator gets a bunch of info from Drive and feeds it into
> Kinematics, whose output it finally feeds into RobotState to be stored
> as a transform that represents an estimate of the Robot's position
> and orientation on the field.

* in teleop, are we operating in setVelocity or in PercentOutput mode?
> it runs open-loop (PercentOutput mode).

* when do they use positionControlMode?
> AIM_TO_GOAL, DRIVE_TOWARDS_GOAL, and TURN_TO_HEADING (in the last case to
> make sure that the encoders are really where they say they are, I think.
> They do this with they gyro in other cases too, and I'm not completely
> sure I understand the rationale. (TBD)

* what units do they use in velocity control mode?
> inches per second

* does their OI allow driver to enter auto-like modes during teleop?
> Yes, it does allow them to enter autonomous like modes > See
> `Drive.setWantAimToGoal()` and the like.

* what’s the difference between aimToGoal and driveTowardsGoal?
> aimToGoal just turns the robot and goes into driveTowardsGoal if
> it's too far away. driveTowardsGoal actually just drives forwards
> until it gets within kShooterOptimalRange.

### Paths
* what is the output format of the cheesypath webapp?  	
> The cheesypath webapp outputs a class that implements PathContainer, and
> has an ArrayList of waypoints which it builds a bath from.

* How are these paths brought into the robot code?
> TBD

### OI
* How do UI events trigger robot actions… are network tables involved?
> Dashboard UI events trigger robot behavior using NetworkTables. Currently,
> this only happens via our `AutoModeSelector`, which is read in Robot.java
> when auto begins.

* If a subsystems wants access to a joystick, how is this obtained?
> Driverstation Joysticks and Buttons are delivered to the robot via the
> standard WPILib conduits.  `Robot:teleopPeriodic` is where we poll buttons
> and joysticks and invoke the appropriate subsystem methods.

* At what point is field position detected for autonomous behavior?
> Field position is never 'detected', but driverstation 'location' is.  
> During `Robot::autonomousInit()` we can employ the `DriverStation`
> methods: `getAlliance()` and  `getGameSpecificMessage()` to obtain
> per-match data.  Note that this information isn't available during
> robot construction.

* Where does joystick remapping occur? what remapping functions are applied?
> There is an interface called `ControlBoardInterface` that allows for
> easy switching between different control schemes (as long as they satisfy
> the interface), but remapping is done with `CheesyDriveHelper` which accepts
> some joystick information from a `ControlBoardInterface` and does a lot of math
> that isn't well motivated in the code. The response seems pretty nice though.

* Which ControlBoardInterface is actually employed?
>  In the unmodified code a class called `ControlBoard` that implements
> `ControlBoardInterface` is used.  A new method has been added that also
> satisfies the interface to get the right mappings for the Xbox controller.

* Who owns the responsibility for inverting the sense of joystick directions?
> Implementors of `ControlBoardInterface` are responsible for mapping the
> knobs and buttons into a canonical form. Directions and mappings should
> remain behind the ControlBoard abstraction.

* How/when/where is the smart dashboard updated?
> The most significant SmartDashboard updates happen when `Robot.allPeriodic()`
> is called. There we invoke `SubsystemManager.outputToSmartDashboard()`,
> which in turn calls that same method its list of all subsystems.
> Note that the Subsystem interface _requires_ all subsystems to implement
> this method.

* Is there a single location/database for all button and joystick identifiers?
> Yes - its the ControlBoardInterface.  Note that each year this interface
> must be changed to reflect control board abstractions (eg: squeeze grabber
> button).  Clients of the interface never rely on specify button mappings
> so it's natural for all the choices to be localized to the code that
> implements the entire interface.


### Vision
254's implementation depended upon an on-board android phone running a custom
vision app and communicating state via a internet-over-usb connection hosted
on the robot side by a port of adb (android debugger). They had a separate
thread that launched and kept the adb connection alive.  They also had a separate
vision control thread to receive data from adb and update robot target state.

### References

DesignNotes.md
