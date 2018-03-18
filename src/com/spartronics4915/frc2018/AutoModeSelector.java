package com.spartronics4915.frc2018;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.spartronics4915.frc2018.auto.AutoModeBase;
import com.spartronics4915.frc2018.auto.modes.*;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that allows a user to select which autonomous mode to execute from the
 * web dashboard.
 */
public class AutoModeSelector
{

    public static final String AUTO_OPTIONS_DASHBOARD_KEY = "AutoStrategyOptions";
    public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "AutoStrategy";

    private static class AutoModeCreator
    {

        private final String mDashboardName;
        private final Supplier<AutoModeBase> mCreator;

        private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator)
        {
            mDashboardName = dashboardName;
            mCreator = creator;
        }
    }

    private static final AutoModeCreator mDefaultMode = new AutoModeCreator(
            "All: Cross Baseline",
            () -> new CrossBaselineMode());
    private static final AutoModeCreator[] mAllModes = {
            new AutoModeCreator("A: Place at Scale", () -> new PlaceScaleFromAMode()),
            new AutoModeCreator("C: Place at Scale", () -> new PlaceScaleFromCMode()),
            new AutoModeCreator("A: Place at Switch", () -> new PlaceSwitchFromAMode()),
            new AutoModeCreator("B: Place at Switch", () -> new PlaceSwitchFromBMode()),
            new AutoModeCreator("C: Place at Switch", () -> new PlaceSwitchFromCMode()),
            new AutoModeCreator("A: Place at Optimized Position", () -> new PlaceOptimizedFromAMode()),
            new AutoModeCreator("C: Place at Optimized Position", () -> new PlaceOptimizedFromCMode()),
            new AutoModeCreator("All: Cross Baseline", () -> new CrossBaselineMode()),
            new AutoModeCreator("Do nothing", () -> new StandStillMode()),
            new AutoModeCreator("Other: Prepare Robot", () -> new PrepareRobotMode()),
            new AutoModeCreator("Test: Path", () -> new TestPathMode()),
            new AutoModeCreator("Test: Velocity PID Tuning", () -> new TestDrivePIDMode("velocity")),
            new AutoModeCreator("Test: Position PID Tuning", () -> new TestDrivePIDMode("position")),
            new AutoModeCreator("Test: Stress Motor Mode", () -> new StressMotorsMode()),
            new AutoModeCreator("Test: Turn to Cube Mode", () -> new TurnToCubeMode()),
            new AutoModeCreator("Test: Find the Cube Mode", () -> new FindCube())

            // e.g. new AutoModeCreator("Boiler Gear then 10 Ball Shoot Red", () -> new BoilerGearThenShootModeRed()),
    };

    public static void initAutoModeSelector()
    {
        Set<String> modesArray = new HashSet<>();
        for (AutoModeCreator mode : mAllModes)
        {
            modesArray.add(mode.mDashboardName);
        }
        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, String.join(",", modesArray));
    }

    public static AutoModeBase getSelectedAutoMode()
    {
        String selectedModeName = SmartDashboard.getString(
                SELECTED_AUTO_MODE_DASHBOARD_KEY,
                "NO SELECTED MODE!!!!");
        Logger.notice("Auto mode name " + selectedModeName);
        for (AutoModeCreator mode : mAllModes)
        {
            if (mode.mDashboardName.equals(selectedModeName))
            {
                return mode.mCreator.get();
            }
        }
        Logger.error("AutoModeSelector failed to select auto mode: " + selectedModeName);
        return mDefaultMode.mCreator.get();
    }
}
