// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.robot.RobotContainer;
//import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.Drivetrain;

public class Move extends CommandBase {
  /** Creates a new Move. */
  double m_time;
  double m_lSpeed;
  double m_rSpeed;
  Drivetrain m_drive;

  public Move(Drivetrain m_drive, double lSpeed, double rSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_lSpeed = lSpeed;
    this.m_rSpeed = rSpeed;
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(-m_lSpeed, -m_rSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(m_lSpeed, m_rSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
