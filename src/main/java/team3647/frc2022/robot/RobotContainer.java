// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2022.autonomous.Auto;
import team3647.frc2022.commands.Move;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import team3647.frc2022.commands.drive;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final XboxController MainController = new XboxController(0);

	private final Drivetrain m_drive = new Drivetrain(Constants.motorLeft, Constants.motorRight);
	private CommandScheduler scheduler = CommandScheduler.getInstance();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		this.scheduler.registerSubsystem(m_drive);
		this.m_drive.setDefaultCommand(new drive(m_drive,MainController::getLeftX, MainController::getLeftY,
				MainController::getRightX, MainController::getRightY, MainController::getXButtonPressed, 
				MainController::getYButtonPressed));
		 
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public SequentialCommandGroup getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		// return m_autoCommand;
		SequentialCommandGroup move = new Auto();
		return move;
		// return null;
	}
}
