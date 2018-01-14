# Porting Notes

* The NavX has a lot of nice resetting features, but we can approximate them with the BNO055
* The library for the NavX, jama (linear algebra), jetty (http), and json are stored in `/lib`
* It appears that you have to install an `adb` binary to `/usr/bin/adb` on the RoboRIO manually. The robot seems to run fine if this isn't present though.
* Autonomous modes run in a thread separate from robot. This is probably why you see the synchronized keyword a lot.
* The member variable naming convention between files is inconsistient. I think this shows the code's age, some is from past years, but I think that they adopted a new convention recently and just didn't change the old stuff? (The new one is `mVariableName` and the old one is `variable_name_`) Update: there's a third convention in some files.
