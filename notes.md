# Porting Notes

* The NavX has a lot of nice resetting features, but we can approximate them with the BNO055
* The library for the NavX, jama (linear algebra), jetty (http), and json are stored in `/lib`
* It appears that you have to install an `adb` binary to `/usr/bin/adb` on the RoboRIO manually. The robot seems to run fine if this isn't present though.
