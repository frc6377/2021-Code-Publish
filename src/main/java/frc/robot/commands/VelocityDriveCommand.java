package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.steering.SteeringMix;
import frc.robot.subsystems.VelocitySubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class VelocityDriveCommand extends CommandBase {
  private final VelocitySubsystem subsystem;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotation;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public VelocityDriveCommand(
      DoubleSupplier forward, DoubleSupplier rotation, VelocitySubsystem subsystem) {
    this.subsystem = subsystem;
    this.forward = forward;
    this.rotation = rotation;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    var drive_wheels = SteeringMix.joystickToTank(forward.getAsDouble(), rotation.getAsDouble());
    subsystem.Drive(drive_wheels);
  }
}
