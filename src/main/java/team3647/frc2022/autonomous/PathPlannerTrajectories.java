// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import team3647.frc2022.commands.SetLED;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.constants.trajConstants;
import team3647.frc2022.subsystems.CANdleSubsystem;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.CANdleSubsystem.AnimationTypes;
import team3647.lib.PPRamseteCommand;

/** Add your docs here. */
public class PathPlannerTrajectories {
        static RamseteController ramseteController = trajConstants.ramseteController;
        public static HashMap<String, Command> eventMap = trajConstants.eventMap;

        public PathPlannerTrajectories() {
        }

        public static enum AutoChoice {
                // AutoList
                AUTO1("Auto1");

                private String autoName;

                // Constructor
                private AutoChoice(String autoName) {
                        this.autoName = autoName;
                }

                // Call trajectory
                private List<PathPlannerTrajectory> getautoTrajectory() {

                        return PathPlanner.loadPathGroup(autoName,
                                        trajConstants.typicalPathConstraints);
                }

        }

        public static Command returnAuto(Drivetrain driveObject, AutoChoice auto) {

                // This will load the file "FullAuto.path" and generate it with a max velocity
                // of 4 m/s and a max acceleration of 3 m/s^2
                // for every path in the group
                List<PathPlannerTrajectory> pathGroup = auto.getautoTrajectory();

                // Event Map Commands
                eventMap.put("set LED Fire", new SetLED(AnimationTypes.Fire, CANdleSubsystem.getInstance()));
                eventMap.put("set LED RgbFade", new SetLED(AnimationTypes.RgbFade, CANdleSubsystem.getInstance()));
                eventMap.put("set LED ColorFlow", new SetLED(AnimationTypes.ColorFlow, CANdleSubsystem.getInstance()));

                // Create the AutoBuilder. This only needs to be created once when robot code
                // starts, not every time you want to create an auto command. A good place to
                // put this is in RobotContainer along with your subsystems.

                RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(driveObject::getPose,
                                driveObject::resetOdometry,
                                ramseteController, driveObject.kinematics, driveObject::drive, eventMap, true,
                                driveObject);

                Command fullAuto = autoBuilder.fullAuto(pathGroup);

                return fullAuto;

        }
}
