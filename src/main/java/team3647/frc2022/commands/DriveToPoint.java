// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.autonomous.PathPlannerTrajectories;
import team3647.frc2022.subsystems.Drivetrain;

public class DriveToPoint extends CommandBase {
  private Drivetrain m_drive;
  private Command followOnFly;

  /** Creates a new DriveToPoint. */
  public DriveToPoint(Drivetrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    followOnFly = PathPlannerTrajectories.onFly(m_drive);
    m_drive.setAuto();
    followOnFly.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setTeleOp();
    followOnFly.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
