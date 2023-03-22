// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.constants.Conversions;
import team3647.lib.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
  WPI_PigeonIMU gyro;
  private double leftVelo;
  private double rightVelo;
  private TalonFX motorLeft;
  private TalonFX motorRight;
  private Field2d field;
  private Pose2d visionPose;
  

  public enum DriveState {
    TELEOP, AUTO
  }

  private DriveState currentState = DriveState.TELEOP;

  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(16));
  DifferentialDrivePoseEstimator odometry;

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

    odometry = new DifferentialDrivePoseEstimator(kinematics, getHeading(), getLeftDist(), getRightDist(),
        new Pose2d());

    field = new Field2d();

    SmartDashboard.putNumber("vision Covariance", 0.9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case TELEOP:
        PathPlannerServer.sendPathFollowingData(getPose(), getPose());
      case AUTO:
        break;
    }

    double motorLeftSelectedSensorVel = motorLeft.getSelectedSensorVelocity();
    double motorRightSelectedSensorVel = motorRight.getSelectedSensorVelocity();

    double motorLeftSelectedSensorPos = motorLeft.getSelectedSensorPosition();
    double motorRightSelectedSensorPos = motorRight.getSelectedSensorPosition();

    SmartDashboard.putNumber("Left Velocity",
        Conversions.stepsPerDecisecondToMetersPerSecond(motorLeftSelectedSensorVel));
    SmartDashboard.putNumber("Right Velocity",
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

    // Limelight Position Compensation
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(Constants.LIMELIGHT_NAME);

    double[] botpose;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      botpose = llresults.targetingResults.botpose_wpiblue;
    } else {
      botpose = llresults.targetingResults.botpose_wpired;
    }
    double pipelineLatency = llresults.targetingResults.latency_pipeline;

    boolean valid = llresults.targetingResults.targets_Fiducials.length > 0;

    visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
    if (valid && Conversions.getSquaredDistance(visionPose.getTranslation(),
        odometry.getEstimatedPosition().getTranslation()) < 1) {
      
      double visionCovariance = SmartDashboard.getNumber("vision Covariance", 0.9);
      odometry.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - (pipelineLatency * 0.001), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionCovariance, visionCovariance, visionCovariance));
    }

    field.setRobotPose(odometry.getEstimatedPosition());

    SmartDashboard.putData("Odometry", field);
  }

  // ---------- FUNCTIONS ----------

  private static Drivetrain m_drive = new Drivetrain();

  public static Drivetrain getInstance() {
    return m_drive;
  }

  public Pose2d getVisionPose () {
    return visionPose;
  }

  public void setPoseToVision () {
    odometry.resetPosition(getHeading(), getLeftDist(), getRightDist(), visionPose);
  }

  public void setAuto() {
    currentState = DriveState.AUTO;
  }

  public void setTeleOp() {
    currentState = DriveState.TELEOP;
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-1 * gyro.getAngle());
  }

  public double getLeftDist() {
    return Conversions.stepsToMeters(motorLeft.getSelectedSensorPosition());
  }

  public double getRightDist() {
    return Conversions.stepsToMeters(motorRight.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(),
        Conversions.stepsToMeters(motorLeft.getSelectedSensorPosition()),
        Conversions.stepsToMeters(motorRight.getSelectedSensorPosition()), pose);
  }

  public void drive(double lSpeed, double rSpeed) {
    this.leftVelo = lSpeed;
    this.rightVelo = rSpeed;
  }

  public void stopMotors() {
    this.leftVelo = 0;
    this.rightVelo = 0;
  }

  // ---------- AUTO ----------
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftVelo, rightVelo);
  }

}
