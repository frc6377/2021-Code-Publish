package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.motorMath.TankDriveMath;
import frc.robot.steering.DriveWheel;
import frc.robot.steering.MotorSetter;

public class VelocitySubsystem extends SubsystemBase implements DriveTrainSubsystem {
  private final MotorSetter rightLeader;
  private final MotorSetter leftLeader;
  private final Config.TankDriveConfig cfg;

  public VelocitySubsystem(
      MotorSetter rightLeader, MotorSetter leftLeader, Config.TankDriveConfig cfg) {
    this.rightLeader = rightLeader;
    this.leftLeader = leftLeader;
    this.cfg = cfg;
  }

  public void Drive(DriveWheel driveWheels) {
    double leftVelocity =
        driveWheels.Left
            * (cfg.controlParameters.velocity
                * 12
                / (TankDriveMath.distancePerTick(
                        cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
                    * 10));
    double rightVelocity =
        driveWheels.Right
            * (cfg.controlParameters.velocity
                * 12
                / (TankDriveMath.distancePerTick(
                        cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
                    * 10));

    rightLeader.set(leftVelocity, 0.0);
    leftLeader.set(rightVelocity, 0.0);
  }
}
