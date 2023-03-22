// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team3647.frc2022.autonomous.Auto;
import team3647.frc2022.autonomous.PathPlannerTrajectories;
import team3647.frc2022.autonomous.PathPlannerTrajectories.AutoChoice;
import team3647.frc2022.commands.Move;
import team3647.frc2022.commands.SetLED;
import team3647.frc2022.commands.Drive;
import team3647.frc2022.commands.DriveToPoint;
import team3647.frc2022.commands.DrivetrainCommands;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.subsystems.CANdleSubsystem;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.CANdleSubsystem.AnimationTypes;

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
	private final CommandXboxController mainController = new CommandXboxController(0);

	public final Drivetrain m_drive = Drivetrain.getInstance();
	private CommandScheduler scheduler = CommandScheduler.getInstance();
	public final CANdleSubsystem canSub = CANdleSubsystem.getInstance();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// this.scheduler.registerSubsystem(m_drive);
		this.m_drive.setDefaultCommand(new Drive(m_drive, mainController::getLeftX, mainController::getLeftY,
				mainController::getRightX, mainController::getRightY,
				mainController.a(), mainController.b(), mainController.x(), mainController.y()));

		this.canSub.setDefaultCommand(new SetLED(AnimationTypes.Rainbow, canSub));
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
		mainController.a().onTrue(
				new Move(m_drive, 0.5, 0.5)
						.until(() -> joystickMoved(mainController))
						.withTimeout(2));
		mainController.y().onTrue(DrivetrainCommands.toggleDriveMode());

		mainController.x().onTrue(new DriveToPoint(m_drive).until(() -> joystickMoved(mainController)));

		mainController.button(8).onTrue(new InstantCommand( () -> {
			m_drive.setPoseToVision();
		}));
	}

	public boolean joystickMoved(CommandXboxController controller) {
		return Math.abs(controller.getLeftX()) > 0.15 ||
				Math.abs(controller.getLeftY()) > 0.15 ||
				Math.abs(controller.getRightX()) > 0.15 ||
				Math.abs(controller.getRightY()) > 0.15;
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return PathPlannerTrajectories.returnAuto(m_drive, AutoChoice.AUTO1);
		// return new Auto(m_drive);
		// return null;
	}
}
