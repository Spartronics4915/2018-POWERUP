package com.spartronics4915.frc2018.paths.profiles;

/**
 * Uses a field and robot profile to calculate Waypoints for the paths used by
 * the GearThenHopperShoot auto modes.
 * 
 * @see RobotProfile
 * @see FieldProfile
 */
public class PathAdapter
{

    static final RobotProfile kRobotProfile = new CompBot();
    static final FieldProfile kFieldProfile = new PracticeField();

    public static RobotProfile getRobotProfile() {
        return kRobotProfile;
    }
    
    public static FieldProfile getFieldProfile() {
        return kFieldProfile;
    }
    
    public static void main(String[] args)
    {
        // You can put print the path out here
    }

    public static void calculatePaths()
    {
        // TODO?
    }

}
