# Build Notes

## Spartronics4915 mods of wpi build
We have installed prereq wpilibs below this repo

* means individual developers don't need wpilibs for _actual_ build
* uniform functionality wrt Travis
* supports building of multiple seasons of code from one environment
* allows us to control wpilib/ctre update timing independent of
  individual developers.

Modification recipe:
1. cp -r ~/wplibs into target repo ($R)
1. modify $R/build.xml (using previous year's template)
    * add support for git version tag
    * add support for build timestamp
    * establish wpilib.root property
    * disallow loading of user's wpilib.properties
1. cp ${wpilib.ant.dir}/build.xml to $R/wpibuild.xml

## Ant build Notes

* ant is really a java application (present in a .jar file on your system)
* ant can be run from the command-line or via an IDE
* $R/build.xml is the main entrypoint for the build.  The wpi build
  foundation is located below ${wpilib.ant.dir}/build.xml.  Since
  we've needed to modify it, we copy it to $R and make our mods there.
* build is controlled by a set of properties
* properties are set first-one-wins
