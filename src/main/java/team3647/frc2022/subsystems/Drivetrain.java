// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.constants.Conversions;

public class Drivetrain extends SubsystemBase {
  WPI_PigeonIMU gyro;
  private double leftVelo;
  private double rightVelo;
  private TalonFX motorLeft;
  private TalonFX motorRight;
  private Field2d field;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(16));
  DifferentialDriveOdometry odometry;

  private static Drivetrain m_drive = new Drivetrain();
  public static Drivetrain getInstance() {
    return m_drive;
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-1 * gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(

    );
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(),
        Conversions.stepsToMeters(motorLeft.getSelectedSensorPosition()),
        Conversions.stepsToMeters(motorRight.getSelectedSensorPosition()), pose);
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    gyro = new WPI_PigeonIMU(16);

    motorLeft = new TalonFX(Constants.MOTOR_LEFT_ID);
    motorRight = new TalonFX(Constants.MOTOR_RIGHT_ID);
    motorLeft.configFactoryDefault();
    motorRight.configFactoryDefault();

    motorLeft.config_kP(0, Constants.DRIVE_kP);
    motorRight.config_kP(0, Constants.DRIVE_kP);

    motorLeft.configVoltageCompSaturation(12);
    motorLeft.enableVoltageCompensation(true);
    motorRight.configVoltageCompSaturation(12);
    motorRight.enableVoltageCompensation(true);

    motorLeft.setInverted(true);
    motorRight.setNeutralMode(Constants.kMasterNeutralMode);
    motorLeft.setNeutralMode(Constants.kMasterNeutralMode);

    odometry = new DifferentialDriveOdometry(getHeading(),
        Conversions.stepsToMeters(motorLeft.getSelectedSensorPosition()),
        Conversions.stepsToMeters(motorRight.getSelectedSensorPosition()));

    field = new Field2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // odometry.update(getHeading(), );
    double motorLeftSelectedSensorVel = motorLeft.getSelectedSensorVelocity();
    double motorRightSelectedSensorVel = motorRight.getSelectedSensorVelocity();

    double motorLeftSelectedSensorPos = motorLeft.getSelectedSensorPosition();
    double motorRightSelectedSensorPos = motorRight.getSelectedSensorPosition();

    SmartDashboard.putNumber("Actual Left Velocity",
        Conversions.stepsPerDecisecondToMetersPerSecond(motorLeftSelectedSensorVel));
    SmartDashboard.putNumber("Actual Right Velocity",
        Conversions.stepsPerDecisecondToMetersPerSecond(motorRightSelectedSensorVel));

    SmartDashboard.putNumber("LeftVel error",
        leftVelo - Conversions.stepsPerDecisecondToMetersPerSecond(motorLeftSelectedSensorVel));
    SmartDashboard.putNumber("RightVel error",
        rightVelo - Conversions.stepsPerDecisecondToMetersPerSecond(motorRightSelectedSensorVel));

    motorLeft.set(ControlMode.Velocity, Conversions.metersToSteps(leftVelo), DemandType.ArbitraryFeedForward,
        Constants.DRIVE_FF.calculate(leftVelo) / 12);
    motorRight.set(ControlMode.Velocity, Conversions.metersToSteps(rightVelo), DemandType.ArbitraryFeedForward,
        Constants.DRIVE_FF.calculate(rightVelo) / 12);

    odometry.update(getHeading(),
        Conversions.stepsToMeters(motorLeftSelectedSensorPos),
        Conversions.stepsToMeters(motorRightSelectedSensorPos));

    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putData("Odometry", field);
  }

  public void drive(double lSpeed, double rSpeed) {
    this.leftVelo = lSpeed;
    this.rightVelo = rSpeed;
  }

  public void stopMotors() {
    this.leftVelo = 0;
    this.rightVelo = 0;
  }
}
