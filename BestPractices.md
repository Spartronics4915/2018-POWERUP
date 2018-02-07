# Best Practices for Spartronics4915 Robot Developers
<!-- TOC depthFrom:3 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [Coding Conventions](#coding-conventions)
- [Logging](#logging)
- [SmartDashboard](#smartdashboard)
- [Subsystem conventions](#subsystem-conventions)
- [Motor conventions](#motor-conventions)
- [Utility functions](#utility-functions)
	- [Fuzzy equality: `boolean Util.epsilonEquals(double a, double b, double epsilon)`](#fuzzy-equality-boolean-utilepsilonequalsdouble-a-double-b-double-epsilon)
	- [Value clamping: `double Util.limit(double val, double min, double max)`](#value-clamping-double-utillimitdouble-val-double-min-double-max)

<!-- /TOC -->
### Coding Conventions
* Member variables: `mVariableName`
* Static member variables: `sVariableName`
* Final static variables: `kVariableName`
* Code must be formatted with built-in eclipse formatter (4 spaces indent, hanging braces)

### Logging
Logging is a very helpful way to diagnose the behavior of your system.  Since some of our
code runs 50 times each second, you must take care to avoid creating a tsunami of log messages.
Not only are they hard to read on the console, but they can actually impact robot performance
since they consume network bandwidth.  Even if the messages aren't visible in the driver station
they may still be impacting performance and stability in a negative fashion. Please follow
these guidelines.

* If you are developing a subsystem, use the Subsystem methods for logging  since it prepends
 your subsystem's name on all logging messages.  `logDebug()` is generally the most valuable
 for debugging. During competition, the messages will not be generated and won't impact performance.  
 logNotice(), logWarning() and logError() should be used in all cases where we want drivers
 (and programmers) aware of malfunction.  

* Since you'll be using the standard log message, you don't need to include your subsystem name.
 So this: `logDebug("closing")` is preferred over this `logDebug("harvester closing")`

* If you are developing outside a Subsystem, you may need to access the Logger directly and
  invoke its associate level-specific methods.  Please make sure to prepend your message text
  with some prefix that makes it obvious where the message came from.

* To avoid logging, you can employ SmartDashboard to broadcast/update a state.

* Logging successful construction, initiation and completion conditions is very helpful for
  on-the-field diagnoses and since these events are occasional, they don't represent any bandwidth
  concerns.  Please use logNotice for all such events.  For example, we log the transition
  into and out of auto and teleop.

### SmartDashboard
SmartDashboard is a simple and powerful tool to communicate bidirectionally between driver station,
roborio and coprocessors like jetson and raspi.  As with logging, SmartDashboard traffic consumes
bandwidth and when overused can negatively impact robot performance.  Some bandwidth-heavy dashboard
state is deemed valuable enough to remain active during competition.  IMU Heading and Drive Odometry
may fall into this category.  Since SmartDashboard entries aren't guaranteed to be visible to driver
station, we must remain vigilant and ensure that only the desired traffic is generated.

* If you are developing a subsystem, please use these methods: `dashboardPutState()`, `dashboardPutString()`,
  `dashboardPutNumber()` and `dashboardPutBoolean()` since they prepend the subsytem name following
  a convenient convention.  

* If your are developing outside a Subsystem, please follow its field-naming convention. Prepend field
  names with a prefix that signifies the message origin.  Our convention is that the prefix follows
  path-like syntax, `MyPrefix/`.  This makes it easy to go to our Dashboard's NetworkTables page an
  enter the prefix string into the filter field.

* SmartDashboard fields become first-class entities when we build a specific presentation for
  them in our web-based GUI.  At this point, names are semi-locked, if you wish to change them
  you must coordinate the change in both robot and dashboard code.

### Subsystem conventions
* use `logInitialized()` to report the state of initialization.
* use `dashboardPutState()` to update the the current subsystem state to the SmartDashboard.
  This method should be invoked during state transitions only, not repeatedly.
* use `dashboardPutWantedState()` when the wanted state changes.


### Motor conventions
that the power isn't wasted and is also delivered in a measurable, consistent fashion.  Motors
are usually driven by the TalonSRX motor controllers.  These are very powerful devices but
also have a large number of mysterious configuration options.  For this reason we employ a
shared _factory_ class, `TalonSRX4915Factory` to construct instances of `TalonSRX4915`.


### Utility functions
There are a number of useful utility functions to help make code more readable and
consistent.  Here are a few samples:

#### Fuzzy equality: `boolean Util.epsilonEquals(double a, double b, double epsilon)`
To compare a target value to a current value with a tolerance for small inaccuracies:
```
    if(Util.epsilonEquals(myPotentiometer.get(), targetValue, .1))
    {
        // we're close enough
    }
```

#### Value clamping: `double Util.limit(double val, double min, double max)`
To constrain an input value to a legal range of values:
```
    double x = Util.limit(myPotentiometer.get(), .1, .4);
```
