// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.constants;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class trajConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    public static PathConstraints typicalPathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared);

    public static RamseteController ramseteController = new RamseteController(trajConstants.kRamseteB,
            trajConstants.kRamseteZeta);
    public static HashMap<String, Command> eventMap = new HashMap<>();

    public static double autoToleranceSquared = 0.05;
}
