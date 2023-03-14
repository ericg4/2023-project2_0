// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.Constants;

public class Drivetrain extends SubsystemBase {
  WPI_PigeonIMU gyro;
  double speedMultiplier = 0.4;
  private double leftVelo;
  private double rightVelo;
  private TalonFX motorLeft;
  private TalonFX motorRight;
  

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(16));
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getAngle(), );

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      
    );
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorLeft = new TalonFX(Constants.MOTOR_LEFT_ID);
    motorRight = new TalonFX(Constants.MOTOR_RIGHT_ID);

    motorLeft.config_kP(0, Constants.DRIVE_kP);
    motorRight.config_kP(0, Constants.DRIVE_kP);

    motorLeft.configVoltageCompSaturation(12);
    motorLeft.enableVoltageCompensation(true);
    motorRight.configVoltageCompSaturation(12);
    motorRight.enableVoltageCompensation(true);

    motorRight.setInverted(true);
    motorRight.setNeutralMode(Constants.kMasterNeutralMode);
    motorLeft.setNeutralMode(Constants.kMasterNeutralMode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //odometry.update(getHeading(), );
    double leftFF = Constants.DRIVE_kS * Math.signum(leftVelo) + Constants.DRIVE_kV * leftVelo + Constants.DRIVE_kA * 0;
    double rightFF = Constants.DRIVE_kS * Math.signum(rightVelo) + Constants.DRIVE_kV * rightVelo + Constants.DRIVE_kA * 0;

    motorLeft.set(ControlMode.Velocity, leftVelo, DemandType.ArbitraryFeedForward, leftFF / 12);
    motorRight.set(ControlMode.Velocity, rightVelo, DemandType.ArbitraryFeedForward, rightFF / 12);

    SmartDashboard.putNumber("encoder somthing", motorLeft.getSelectedSensorVelocity());
  }

  public void drive(double lSpeed, double rSpeed) {
    this.leftVelo = lSpeed * speedMultiplier;
    this.rightVelo = rSpeed * speedMultiplier;
  }

  public void stopMotors() {
    this.leftVelo = 0;
    this.rightVelo = 0;
  }
}
