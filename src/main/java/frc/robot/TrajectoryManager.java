// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;

/** Add your docs here. */
public class TrajectoryManager {

    public Trajectory testPath = PathPlanner.loadPath("Test Path", 0.1, 0.1);
    public Trajectory shortPath = PathPlanner.loadPath("Short Path", 2, 2);

}
