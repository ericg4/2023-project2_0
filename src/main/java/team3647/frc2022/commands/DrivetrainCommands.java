// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team3647.frc2022.subsystems.Drivetrain;

/** Add your docs here. */
public class DrivetrainCommands {
    private final Drivetrain m_drive;
    private final CommandXboxController m_Controller;

    public DrivetrainCommands(Drivetrain m_drive, CommandXboxController m_Controller) {
        this.m_drive = m_drive;
        this.m_Controller = m_Controller;
    }

    public static Command toggleDriveMode() {
        return Commands.runOnce(
                () -> Drive.driveMode = !Drive.driveMode);
    }
}
