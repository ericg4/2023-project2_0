// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class DrivetrainCommands {

    public DrivetrainCommands() {}

    public static Command toggleDriveMode() {
        return Commands.runOnce(
                () -> Drive.driveMode = !Drive.driveMode);
    }
}
