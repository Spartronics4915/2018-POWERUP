# Spartronics 4915 modified clone of Team 254's 2017 Codebase

We've cloned the 254 codebase and modified it to better match our team's
hardware, vision, programming and electronics conventions. We've also ported 
it to the 2018 WPI and CTRE interfaces.  Code can be built via command-line
ant or your favorite Java IDE (Eclipse, IntelliJ, ...).

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

For more details on code highlights, please refer to CheesyNotes.md.

