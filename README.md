# 2018-POWERUP [![Build Status](https://travis-ci.com/Spartronics4915/2018-POWERUP.svg?token=fyEWjwiNdUZ8u7W5snBy&branch=master)](https://travis-ci.com/Spartronics4915/2018-POWERUP)

Welcome to the github for Spartronics code for the FRC 2018-POWERUP game.

### Helpful Links

[Best Practices](BestPractices.md) |
[WPILib](http://wpilib.screenstepslive.com/s/4485) |
[WPILib Java Ref](http://first.wpi.edu/FRC/roborio/release/docs/java) |
[CTRE Java Ref](http://www.ctr-electronics.com/downloads/api/java/html/index.html) |
[CANTalon User's Guide](http://www.ctr-electronics.com/Talon%20SRX%20User's%20Guide.pdf) |
[CANTalon Ref Guide](http://www.ctr-electronics.com/Talon%20SRX%20Software%20Reference%20Manual.pdf) |
[Travis CI](https://travis-ci.com/Spartronics4915/2018-POWERUP)

[Learning this Codebase](LearningCodebase.md) |
[Spartronics Developer Handbook](https://binnur.gitbooks.io/spartronics-developers-handbook/content/) |
[Spartronics Intro Slides](https://docs.google.com/presentation/d/1ZiMBC9y3xrwFk1akdaiV_BMLLS6EyY6BSfiTRQo1KlM/edit#slide=id.g190898ba99_1_437)

## Code Ancestry/Acknowledgements
This is a derivative of team 254's 2017 codebase, modified to better match our
team's hardware, vision, programming and electronics conventions. We've also
ported it to the 2018 WPI and CTRE interfaces.  Code can be built via
command-line ant or your favorite Java IDE (Eclipse, IntelliJ, ...).

Modifications include:
* refactor into our namespaces
* remove 2017-specific game code
* support for pigeon imu
* disabled vision/adb connection to android phone
* port to 2018 wpi/ctre interfaces
* modified ant build for better Travis building
* removed dash and dash_release
* removed vision_app
* modified build system:
    * embed version and developer info into app manifest
    * hardcode refs to support library to wpilib/... in this repo
    * added test target and minimal travis/slack support

Many thanks to CheesyFolks (Team254) for sharing your code!

For more details on code highlights, please refer to
[CheesyNotes](CheesyNotes.md).
