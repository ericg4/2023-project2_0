// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.Drivetrain;

public class drive extends CommandBase {
  /** Creates a new drive. */
  DoubleSupplier supLeftX;
  DoubleSupplier supLeftY;
  DoubleSupplier supRightX;
  DoubleSupplier supRightY;
  BooleanSupplier xButton;
  BooleanSupplier yButton;
  Drivetrain m_drive;
  private boolean driveMode = false;

  public drive(Drivetrain m_drive, DoubleSupplier supLeftX, DoubleSupplier supLeftY, DoubleSupplier supRightX, DoubleSupplier supRightY, BooleanSupplier xButton, BooleanSupplier yButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.supLeftX = supLeftX;
    this.supLeftY = supLeftY;
    this.supRightX = supRightX;
    this.supRightY = supRightY;
    this.xButton = xButton;
    this.yButton = yButton;
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xButton.getAsBoolean()) {
      if (driveMode) {
        driveMode = false;
        System.out.println("TankDrive");
      }
      else {
        driveMode = true;
        System.out.println("ArcadeDrive");
      }
    }

    if (!driveMode) {
      tankDrive();
    }
    else {
      arcadeDrive();
    }
  }

  public void tankDrive() {
    m_drive.setLeftMotors(supLeftY.getAsDouble());
    m_drive.setRightMotors(supRightY.getAsDouble());
  }

  public void arcadeDrive() {
    m_drive.setLeftMotors(supLeftY.getAsDouble() - supLeftX.getAsDouble());
    m_drive.setRightMotors(supLeftY.getAsDouble() + supLeftX.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
