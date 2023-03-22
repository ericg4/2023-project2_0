// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

//import team3647.frc2022.subsystems.Drivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final NeutralMode kMasterNeutralMode = NeutralMode.Brake;
    public static final int MOTOR_LEFT_ID = 01;
    public static final int MOTOR_RIGHT_ID = 02;

    public static final double WHEEL_CIRCUMFERENCE_INCHES = 4 * Math.PI;
    public static final double GEARING = 1.0 / 5.0;
    public static final double ENCODER_COUNT = 2048;

    public static final double DRIVE_kP = 0.00044612;
    public static final double DRIVE_kS = 0.073355;
    public static final double DRIVE_kV = 1.7502;
    public static final double DRIVE_kA = 0.19031;

    public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV);
    public static final double DRIVE_MULTIPLIER = 4;

    public static final WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);

    public static final int BUTTON_Y = 0;
    public static final int CANdleID = 4;
    public static final String LIMELIGHT_NAME = "limelight-right";

}
