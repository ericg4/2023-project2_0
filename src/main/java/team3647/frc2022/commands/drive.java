// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.Drivetrain;

public class Drive extends CommandBase {
  /** Creates a new drive. */
  DoubleSupplier supLeftX;
  DoubleSupplier supLeftY;
  DoubleSupplier supRightX;
  DoubleSupplier supRightY;
  BooleanSupplier aButton;
  BooleanSupplier bButton;
  BooleanSupplier xButton;
  BooleanSupplier yButton;
  Drivetrain m_drive;
  public static boolean driveMode = false;
  boolean moving = false;

  public Drive(Drivetrain m_drive, DoubleSupplier supLeftX, DoubleSupplier supLeftY, DoubleSupplier supRightX,
      DoubleSupplier supRightY, BooleanSupplier aButton, BooleanSupplier bButton, BooleanSupplier xButton,
      BooleanSupplier yButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.supLeftX = supLeftX;
    this.supLeftY = supLeftY;
    this.supRightX = supRightX;
    this.supRightY = supRightY;
    this.aButton = aButton;
    this.bButton = bButton;
    this.xButton = xButton;
    this.yButton = yButton;
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
    // Driving with sticks
    if (!driveMode) {
      tankDrive();
    } else {
      arcadeDrive();
    }
  }

  public void tankDrive() {
    m_drive.drive(modifyInputs(supLeftY.getAsDouble()),
        modifyInputs(supRightY.getAsDouble()));
  }

  public void arcadeDrive() {
    m_drive.drive(modifyInputs(supLeftY.getAsDouble()) - modifyInputs(supRightX.getAsDouble()), 
      modifyInputs(supLeftY.getAsDouble()) + modifyInputs(supRightX.getAsDouble()));
  }

  public double modifyInputs(double input) {
    input = Math.abs(input) > 0.15 ? Math.signum(input) * Math.pow(input, 2) : 0;
    return input * Constants.DRIVE_MULTIPLIER * Constants.SPEED_COMPENSATION * -1;
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
