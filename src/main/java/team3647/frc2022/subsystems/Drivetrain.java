// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  TalonFX motorRight;
  TalonFX motorLeft;
  double speedMultiplier = 0.4;
  private double leftVelo;
  private double rightVelo;

  /** Creates a new Drivetrain. */
  public Drivetrain(TalonFX left, TalonFX right) {
    motorLeft = left;
    motorRight = right;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorLeft.set(ControlMode.PercentOutput, leftVelo);
    motorRight.set(ControlMode.PercentOutput, rightVelo);
  }

  public void setLeftMotors(double speed) {
    this.leftVelo = speed*speedMultiplier;
  }
  public void setRightMotors(double speed) {
    this.rightVelo = speed*speedMultiplier;
  }
  public void drive(double lSpeed, double rSpeed) {
    this.leftVelo = lSpeed*speedMultiplier;
    this.rightVelo = rSpeed*speedMultiplier;
  }
  public void stopMotors() {
    this.leftVelo = 0;
    this.rightVelo = 0;
  }
}
