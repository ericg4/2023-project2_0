// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import team3647.frc2022.commands.SetLED;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.constants.trajConstants;
import team3647.frc2022.subsystems.CANdleSubsystem;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.CANdleSubsystem.AnimationTypes;

/** Add your docs here. */
public class PathPlannerTrajectories {
    static RamseteController ramseteController = new RamseteController(trajConstants.kRamseteB,
            trajConstants.kRamseteZeta);

    public PathPlannerTrajectories() {
    }

    public static final PathPlannerTrajectory auto1 = PathPlanner.loadPath("Auto1",
            trajConstants.kMaxSpeedMetersPerSecond, trajConstants.kMaxAccelerationMetersPerSecondSquared);
    public static final Pose2d startState1 = auto1.getInitialPose();

    public static Command returnAuto(Drivetrain driveObject) {
        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Auto1",
                trajConstants.kMaxSpeedMetersPerSecond, trajConstants.kMaxAccelerationMetersPerSecondSquared);

        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("set LED Fire", new SetLED(AnimationTypes.Fire, CANdleSubsystem.getInstance()));
        eventMap.put("set LED RgbFade", new SetLED(AnimationTypes.RgbFade, CANdleSubsystem.getInstance()));
        eventMap.put("set LED ColorFlow", new SetLED(AnimationTypes.ColorFlow, CANdleSubsystem.getInstance()));

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.

        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(driveObject::getPose, driveObject::resetOdometry,
                ramseteController, driveObject.kinematics, driveObject::drive, eventMap, true, driveObject);

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        return fullAuto;

    }
}
