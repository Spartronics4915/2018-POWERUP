package com.spartronics4915.frc2018.loops;

import com.spartronics4915.frc2018.GoalTracker;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.vision.VisionUpdate;
import com.spartronics4915.frc2018.vision.VisionUpdateReceiver;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in RobotState. This helps
 * keep track of goals detected by the vision system. The code to determine the best goal to shoot
 * at and prune old Goal tracks is in GoalTracker.java
 *
 * @see GoalTracker.java
 */
public class VisionProcessor implements Loop, VisionUpdateReceiver {
  static VisionProcessor instance_ = new VisionProcessor();
  VisionUpdate update_ = null;
  RobotState robot_state_ = RobotState.getInstance();

  public static VisionProcessor getInstance() {
    return instance_;
  }

  VisionProcessor() {}

  @Override
  public void onStart(double timestamp) {}

  @Override
  public void onLoop(double timestamp) {
    VisionUpdate update;
    synchronized (this) {
      if (update_ == null) {
        return;
      }
      update = update_;
      update_ = null;
    }
    robot_state_.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
  }

  @Override
  public void onStop(double timestamp) {
    // no-op
  }

  @Override
  public synchronized void gotUpdate(VisionUpdate update) {
    update_ = update;
  }
}
