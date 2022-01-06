package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.motorMath.TankDriveMath;
import frc.robot.steering.MotorSetter;

public class DifferentialSubsystem extends SubsystemBase implements DriveTrainSubsystem {
  private final MotorSetter rightLeader;
  private final Config.TankDriveConfig cfg;

  public DifferentialSubsystem(MotorSetter rightLeader, Config.TankDriveConfig cfg) {
    this.rightLeader = rightLeader;
    this.cfg = cfg;
  }

  public static class DriveVelocity {
    public double Forward;
    public double Left;

    public DriveVelocity(double forward, double left) {
      Forward = forward;
      Left = left;
    }
  }

  public void Drive(DriveVelocity driveVelocity) {
    double demand0 =
        cfg.controlParameters.velocity
            * 12
            / (TankDriveMath.distancePerTick(
                cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter))
            * driveVelocity.Forward;
    double demand1 = driveVelocity.Left * 3600;
    rightLeader.set(demand0, demand1);
  }
}
