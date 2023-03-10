// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.Drivetrain;

public class Move extends CommandBase {
  /** Creates a new Move. */
  double m_time;
  double m_lSpeed;
  double m_rSpeed;
  Drivetrain m_drive;
  Timer timer = new Timer();
  
  public Move(Drivetrain m_drive, double time, double lSpeed, double rSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_time = time;
    this.m_lSpeed = lSpeed;
    this.m_rSpeed = rSpeed;
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setLeftMotors(m_lSpeed);
    m_drive.setLeftMotors(m_rSpeed);

    if (timer.get() >= m_time) {
      end(true);
      System.out.println("STOPPED AT " + timer.get() + "s");
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setLeftMotors(0);
    m_drive.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
