## Learning Spartronics-254Base

### Basic execution model, bootstrapping, debugging

* how does our code get built, downloaded and launched?
> our build system relies on `ant` which describes the build process
> with a series of .xml and .properties files.  Ultimately it
> creates a file named `FRCUserProgram.jar`. This file and associated
> runtime libraries are copied down to the robot (to /home/lvuser and
> to /usr/local/frc/lib).
> Another file, `robotCommand` (or `robotDebugCommand`) is also copied.  
> This file is *NOT* a shell script but rather a command file that is parsed by
> `/usr/local/frc/bin/frcRunRobot.sh` and ultimately passed to
> `/sbin/start-stop-daemon` whose job is to 'watch over'
> the robot process - to restart it if it crashes, to start it on robot
> reboot, etc.  When we deploy a robot-program that crashes, the start-stop-daemon
> can produce an annoying and confusing flurry of program restarts.  The only
> way to stop the flurry is either to kill the process group or to deploy a
> stable robot program.  The output of the robot program is also routed over the
> network via this startup machinery.

* where are crashlogs found?  How did they get there?
> /home/lvuser/crash_tracking.txt is created by Logger (formerly CrashTracker)
> There's also a log of program output produced by the bootstrapping mechanism
> located here: `/var/local/natinst/log/FRC_UserProgram.log`. It may be
> helpful to create a symbolic link to this file in the lvuser home.

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

* What camera did 254 use? How were vision targets delivered to the robot
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

* What threads are involved in delivering vision targets?
> `VisionProcessor` implements Loop and VisionUpdateReceiver.  Its job is to
> to deliver VisionUpdates (received via synchronized gotUpdate())
> to RobotState.  `RobotStateEstimator` is also running in a separate thread
> and computes/delivers updates to the RobotState based on the `Drive` sensors.
> Note that VisionProcessor sends data directly to RobotState in a manner
> analogous to the RobotStateEstimator.  In that sense, the VisionProcessor
> can be thought of as a `GoalStateEstimator`.

* What is the processing required to act upon vision target acquisition?
> Remember that the goal is captured in the coordinate frame of the camera
> and must be converted through the coordinate frame of the robot
> _at the time of capture_ (ie in the past) to the field coordinate frame,
> then back to the _current_ robot coordinate frame in order to inform robot
> motion planning.

* Why is `TargetInfo`'s X coordinate always zero?
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

### RobotStateEstimator

* What is the job of `RobotStateEstimator`?
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

* What is the purpose of `RobotState`?
> RobotState keeps track of the poses of various coordinate frames throughout
> the match. A coordinate frame is simply a point and direction in space that
> defines an (x,y) coordinate system. Transforms (or poses) keep track of the
> spatial relationship between different frames.

* What are its frames of interest?
> * `Field frame`: origin is where the robot is turned on
> * `Vehicle frame`: origin is the center of the robot wheelbase, facing
>    forwards. This frame is relative to the field frame.
> * `Camera frame`: origin is the center of the camera imager relative
>  to the robot frame.
> * `Goal frame`: origin is the center of the target (note that orientation in
> this frame is arbitrary). Also note that there can be multiple target frames.

* What is a `kinematic chain`?
> A linked series of transformations from one frame to another is known as
> a kinematic chain. In the "forward" direction we can start with a position
> on the field to compute, for example, the position of a goal.  If we're
> given a position in the field frame, we can compute, for example, its coordinates
> in the camera frame by performing coordinate conversion in the _forward_
> direction.  If we're given a position in the camera frame, we convert that
> to the goal frame by performing coordinate conversion in the _inverse_
> direction. Ours is a kinematic chain with 4 frames, and so there are 3
> transforms of interest:
> 1. Field-to-vehicle: This is tracked over time by integrating encoder and
> gyro measurements. It will inevitably drift, but is usually accurate over
> short time periods.
> 2. Vehicle-to-camera: This is a constant.
> 3. Camera-to-goal: This is a pure translation, and is measured by the vision
>   system.

* How do we convert coordinates between frames of interest?
> `RigidTransform2d` is the class responsible for transforming points
> from one coordinate frame to another. It is composed of a Translation2d
> and a Rotation2d.  Rotation2d is represented as an angle (which is decomposed
> into cos_theta and sin_theta, presumably for performance).  Another geometric
> type is Twist2d which is used to represent movement along an arc of constant
> curvature at a constant velocity. We can apply a Twist2d to a RigidTransform2d
> to produce a new RigidTransform2d and this represents a form of incremental
> (turning) motion.

* How is it we can look up the position of the robot at any point in the past?
> `RobotState` maintains a sorted list of robot positions:
```
private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> mFieldToVehicle;
```
> where the key to this map is the timestamp associated with the robot's
> position on the field (again, relative to its starting location and
> orientation). Each timeslice, RobotStateEstimator issues a call to
> `addObservation` to append a new sample to this map.

* Why do we care about past robot position and orientation?
> Because of fundamental latencies in our vision sampling and processing
> targets are actually found relative to a past position.  In order to
> understand the target location in terms of current robot position,
> we must compute the target position in field coordinates, then convert
> them to the _current_ robot frame.

### Paths
* what is the output format of the cheesy_path webapp?  	
> The cheesypath webapp outputs java code suitable for inclusion in a robot
> project.  The code takes the form of a custom class that implements PathContainer,
> with an `ArrayList<WayPoint>` representing the results of user interaction.

* How are these paths brought into the robot code?
> The results of cheesy_path are included in robot code by adding the
> custom class java file directly to the repository.  By convention, custom
> PathContainers are placed in a game-specific area: `frc2018/paths`.  
> Next an instance of this class can be created as a component of an
> autonomous mode.  The file, `frc2018/auto/modes/TestPathMode` is an
> example that instantiates the class TestPath and instantiates an instance
> of `DrivePathAction`, parameterized by the TestPath instance.

* How are these paths used to affect the drive trajectory?
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

* What is PathFollower and how does it work?
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

* What is an AdaptivePurePursuitController?
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

* What is a Path?
> A Path is a List<PathSegment>.  The current segment represents a part of
> the entire path that we are currently following. When we arrive at the end
> of the current segment, it is removed from the list and we adopt a new
> current segment.  The current segment is always at index 0 of our list and
> we know we've reached our target when the PathSegment list is empty. Presumably,
> a Path has geometric continuity at segment boundaries. This constraint must
> be enforced by the Path authoring system.

* What is a PathSegment?
> A PathSegment is a component of a larger path that represents either a
> linear or circular arc with target speeds at each endpoint. In addition
> to the geometric descriptors, a PathSegment also has-a `MotionProfile`
> named `speedController`. Central to the functionality of PathSegment are
> these methods:
> * `getClosestPoint()`
> * `getPointByDistance()`
> * `getSpeedByClosestPoint()`
> * `getSpeedByDistance()`

* What is a MotionProfile?
> from code comments:
```
 /**
 * A motion profile specifies a 1D time-parameterized trajectory. The trajectory
 * is composed of successively coincident MotionSegments from which the desired
 * state of motion at any given distance or time can be calculated.
 */
```

* What is a MotionSegment?
> from code comments:
```
 /**
 * A MotionSegment is a movement from a start MotionState to an end MotionState
 * with a constant acceleration.
 */
```

* What is a MotionState?
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

DesignNotes.md
