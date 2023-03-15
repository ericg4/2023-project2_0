// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2022.commands.Move;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public Auto(Drivetrain m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Move(m_drive, 0.5, 0.5).withTimeout(2),
        new Move(m_drive, -0.5, -0.5).withTimeout(2),
        new Move(m_drive, -0.5, 0.5).withTimeout(1));
  }
}
