package com.team254.frc2017.paths.profiles;

/**
 * Uses a field and robot profile to calculate Waypoints for the paths used by the GearThenHopperShoot auto modes.
 * 
 * @see RobotProfile
 * @see FieldProfile
 */
public class PathAdapter {

    static final RobotProfile kRobotProfile = new CompBot();
    static final FieldProfile kFieldProfile = new PracticeField();

    // Path Variables -- these will probably be common for many paths if you're only changing directions
    static final double kLargeRadius = 45;
    static final double kModerateRadius = 30;
    static final double kNominalRadius = 20;
    static final double kSmallRadius = 10;
    static final double kSpeed = 80;

    public static void calculatePaths() {
        // TODO: Put paths here
    }

    public static void main(String[] args) {
        // You can put print the path out here
    }

}
