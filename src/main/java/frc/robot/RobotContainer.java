// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.Config;
import frc.robot.setup.Hardware;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import io.github.oblarg.oblog.Logger;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrainSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Hardware hardware;
  private final Config config;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Config cfg, Hardware hardware) {
    Logger.configureLoggingAndConfig(this, false);

    this.hardware = hardware;
    this.config = cfg;
    this.driveSubsystem = this.hardware.TrainHardware.DriveTrainSubsystem;

    // Configure the button bindings
    configureButtonBindings(this.hardware.DriverController, this.hardware.GunnerController);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveSubsystem.setDefaultCommand(this.hardware.TrainHardware.DefaultCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(XboxController driver, XboxController gunner) {
    new JoystickButton(driver, Button.kBumperLeft.value)
        .whenPressed(() -> intakeSubsystem.Deploy())
        .whenReleased(() -> intakeSubsystem.Retract());

    BooleanSupplier driverLeftTrigger =
        () -> driver.getTriggerAxis(Hand.kLeft) >= config.Driver.triggerSensitivity;
    new Trigger(driverLeftTrigger)
        .whenActive(() -> intakeSubsystem.IntakeCells(true))
        .whenInactive(() -> intakeSubsystem.IntakeCells(false));

    BooleanSupplier driverRightTrigger =
        () -> driver.getTriggerAxis(Hand.kRight) >= config.Driver.triggerSensitivity;
    new Trigger(driverRightTrigger)
        .whenActive(() -> intakeSubsystem.EjectCells(true))
        .whenInactive(() -> intakeSubsystem.EjectCells(false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
