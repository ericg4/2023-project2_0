// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import team3647.frc2022.subsystems.Drivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final NeutralMode kMasterNeutralMode = NeutralMode.Brake;
    public static final int MOTOR_LEFT_ID = 01;
    public static final int MOTOR_RIGHT_ID = 02;

    public static final TalonFX motorLeft = new TalonFX(MOTOR_LEFT_ID);
    public static final TalonFX motorRight = new TalonFX(MOTOR_RIGHT_ID);

    //public static final Drivetrain m_drive = new Drivetrain(motorLeft, motorRight);

    public static final int BUTTON_Y = 0;
    

    static {
        motorRight.setInverted(true);
        motorRight.setNeutralMode(kMasterNeutralMode);
        motorLeft.setNeutralMode(kMasterNeutralMode);
    }

}
