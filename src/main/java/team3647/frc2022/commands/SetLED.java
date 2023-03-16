// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3647.frc2022.subsystems.CANdleSubsystem;
import team3647.frc2022.subsystems.CANdleSubsystem.AnimationTypes;

public class SetLED extends InstantCommand {
  /** Creates a new SetLED. */
  private CANdleSubsystem candleSub;
  private AnimationTypes anType;

  public SetLED(AnimationTypes anType, CANdleSubsystem candleSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.candleSub = candleSub;
    this.anType = anType;

    addRequirements(candleSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candleSub.changeAnimation(anType);
  }
}
