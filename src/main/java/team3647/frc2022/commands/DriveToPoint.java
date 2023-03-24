// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.autonomous.PathPlannerTrajectories;
import team3647.frc2022.constants.Conversions;
import team3647.frc2022.constants.trajConstants;
import team3647.frc2022.subsystems.CANdleSubsystem;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.lib.PPRamseteCommand;

public class DriveToPoint extends CommandBase {
  private Drivetrain m_drive = Drivetrain.getInstance();
  private Command followOnFly;
  private Timer timer = new Timer();
  private CANdleSubsystem LEDs = CANdleSubsystem.getInstance();

  /** Creates a new DriveToPoint. */
  public DriveToPoint() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    followOnFly = onFly(m_drive, traj1);
    m_drive.setAuto();
    followOnFly.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LEDs.setToColor(variableColor(traj1), 0, 4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setTeleOp();
    followOnFly.end(interrupted);
    System.out.println("Ended DriveToPoint");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Paths
  Pose2d startingPose = m_drive.getPose();
  PathPlannerTrajectory traj1 = PathPlanner.generatePath(
      new PathConstraints(1, 0.5),
      new PathPoint(startingPose.getTranslation(), startingPose.getRotation()), // position, heading
      new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(180)) // position, heading
  );

  public static Command onFly(Drivetrain driveObject, PathPlannerTrajectory path) {
    return new PPRamseteCommand(path, driveObject::getPose, trajConstants.ramseteController,
        driveObject.kinematics, driveObject::drive, driveObject);
  }

  public double percentOfWay(PathPlannerTrajectory path) {
    double totalTime = path.getTotalTimeSeconds();
    return timer.get() * 1.0 / totalTime;
  }

  public int[] variableColor (PathPlannerTrajectory path) {
    int[] rgbColors = new int[3];
    double dist = percentOfWay(path);
    if (dist > 1) {
      dist = 1;
    }
    rgbColors[0] = 255 - (int) (dist * 255);
    rgbColors[1] = 0;
    rgbColors[2] = (int) (dist * 255);

    return rgbColors;
  }
}
