package frc.robot.setup;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.Config;

public class Hardware {
  public XboxController DriverController;
  public XboxController GunnerController;
  public DriveTrainHardware TrainHardware;

  public Hardware(Config cfg) {
    DriverController = new XboxController(cfg.Driver.joystickPortID);
    GunnerController = new XboxController(cfg.Gunner.joystickPortID);
    TrainHardware = new DriveTrainHardware(DriverController, cfg.TankDrive);
  }
}
