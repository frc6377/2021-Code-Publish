package frc.robot.setup;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.DifferentialDriveCommand;
import frc.robot.commands.VelocityDriveCommand;
import frc.robot.config.Config;
import frc.robot.motorMath.TankDriveMath;
import frc.robot.steering.MotorSetter;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VelocitySubsystem;
import java.util.function.DoubleSupplier;

public class DriveTrainHardware {
  public TalonFX driveLeftLead = null;
  public TalonFX driveRightLead = null;
  public TalonFX driveLeftFollow = null;
  public TalonFX driveRightFollow = null;
  public PigeonIMU pigeon = null;
  public DriveTrainSubsystem DriveTrainSubsystem = null;
  public CommandBase DefaultCommand = null;

  public DriveTrainHardware(XboxController controller, Config.TankDriveConfig cfg) {
    // Don't initialize hardware for a simulated drivetrain
    if (cfg.simulated) {
      return;
    }

    pigeon = new PigeonIMU(cfg.pigeonID);
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);

    switch (cfg.controlParameters.method) {
      case DIFFERENTIAL:
        DifferientalDriveTrain(controller, cfg);
        return;
      default:
        VelocityDriveTrain(controller, cfg);
        return;
    }
  }

  public void DifferientalDriveTrain(XboxController controller, Config.TankDriveConfig cfg) {
    DifferentialDrivetrainMotorConfig(cfg);
    MotorSetter demandSet =
        (double demand0, double demand1) ->
            driveRightLead.set(TalonFXControlMode.Velocity, demand0, DemandType.AuxPID, demand1);
    var driveTrain = new DifferentialSubsystem(demandSet, cfg);
    DriveTrainSubsystem = driveTrain;
    DoubleSupplier forward = () -> 0 - controller.getY(Hand.kLeft);
    DoubleSupplier rotation = () -> 0 - controller.getX(Hand.kRight);
    DefaultCommand = new DifferentialDriveCommand(forward, rotation, driveTrain);
  }

  public void VelocityDriveTrain(XboxController controller, Config.TankDriveConfig cfg) {
    VelocityDrivetrainMotorConfig(cfg);
    var driveTrain =
        new VelocitySubsystem(
            (double value, double unused) -> driveRightLead.set(TalonFXControlMode.Velocity, value),
            (double value, double unused) -> driveLeftLead.set(TalonFXControlMode.Velocity, value),
            cfg);
    DriveTrainSubsystem = driveTrain;
    DoubleSupplier forward = () -> 0 - controller.getY(Hand.kLeft);
    DoubleSupplier rotation = () -> 0 - controller.getX(Hand.kRight);
    DefaultCommand = new VelocityDriveCommand(forward, rotation, driveTrain);
  }

  public void VelocityDrivetrainMotorConfig(Config.TankDriveConfig cfg) {
    TalonFX[] right = configureVelocitySide(cfg, cfg.rightLeaderID, cfg.rightFollowerID, true);
    TalonFX[] left = configureVelocitySide(cfg, cfg.leftLeaderID, cfg.leftFollowerID, false);
    driveRightLead = right[0];
    driveRightFollow = right[1];
    driveLeftLead = left[0];
    driveLeftFollow = left[1];
  }

  public void DifferentialDrivetrainMotorConfig(Config.TankDriveConfig cfg) {
    // configure differential encoder mode
    var leftLeaderConfig = new motorfactory.MotorConfiguration();
    var leftFollowConfig = new motorfactory.MotorConfiguration();
    var rightLeaderConfig = new motorfactory.MotorConfiguration();
    var rightFollowConfig = new motorfactory.MotorConfiguration();

    // set inverts
    rightInvert(cfg, rightLeaderConfig);
    rightInvert(cfg, rightFollowConfig);
    leftInvert(cfg, leftLeaderConfig);
    leftInvert(cfg, leftFollowConfig);

    // Set feedback sensor config
    leftLeaderConfig.talonFXConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    rightLeaderConfig.talonFXConfig.remoteFilter0.remoteSensorDeviceID = cfg.leftLeaderID;
    rightLeaderConfig.talonFXConfig.remoteFilter0.remoteSensorSource =
        RemoteSensorSource.TalonFX_SelectedSensor;
    rightLeaderConfig.talonFXConfig.remoteFilter1.remoteSensorDeviceID = cfg.pigeonID;
    rightLeaderConfig.talonFXConfig.remoteFilter1.remoteSensorSource =
        RemoteSensorSource.Pigeon_Yaw;

    /* Now that the Left sensor can be used by the leader Talon,
     * set up the Left (Aux) and Right (Leader) distance into a single
     * Robot distance as the Leader's Selected Sensor 0. */
    setRobotDistanceConfigs(rightLeaderConfig.invertType, rightLeaderConfig.talonFXConfig);

    // setup difference signal to be used for turn when performing drive straight with encoders
    double ticksPerRobotSpin =
        TankDriveMath.ticksPerRobotSpin(
            cfg.physicalLayout.wheelBase,
            cfg.physicalLayout.wheelDiameter,
            cfg.physicalLayout.gearRatio);
    setRobotTurnConfigs(
        rightLeaderConfig.invertType, rightLeaderConfig.talonFXConfig, (int) ticksPerRobotSpin);

    setMotionMagicConfigs(cfg, rightLeaderConfig.talonFXConfig);
    setDiffVelocityPidConfigs(cfg, rightLeaderConfig.talonFXConfig);
    setTurnPidConfigs(cfg, rightLeaderConfig.talonFXConfig);

    driveRightLead = motorfactory.createTalonFX(cfg.rightLeaderID, rightLeaderConfig);
    driveRightFollow =
        motorfactory.createTalonFX(cfg.rightFollowerID, rightFollowConfig, driveRightLead);
    driveLeftLead =
        motorfactory.createTalonFX(
            cfg.leftLeaderID, leftLeaderConfig, driveRightLead, FollowerType.AuxOutput1);
    driveLeftFollow =
        motorfactory.createTalonFX(
            cfg.leftFollowerID, leftFollowConfig, driveRightLead, FollowerType.AuxOutput1);
    driveRightLead.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 60);
    driveRightLead.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 60);
    driveRightLead.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 60);
    driveRightLead.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, 60);
    driveLeftLead.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 60);
    switchToPigeon();
    switchToEncoder(cfg);
  }

  public TalonFX[] configureVelocitySide(
      Config.TankDriveConfig cfg, int leadID, int followID, boolean isRight) {
    var leadConfig = new motorfactory.MotorConfiguration();
    var followConfig = new motorfactory.MotorConfiguration();

    // set inverts
    if (isRight) {
      rightInvert(cfg, leadConfig);
    } else {
      leftInvert(cfg, leadConfig);
    }
    followConfig.invertType = TalonFXInvertType.FollowMaster;

    // set integrated sensor
    leadConfig.talonFXConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // configure current limits - drivetrains are usually on 40A breakers
    leadConfig.talonFXConfig.supplyCurrLimit = motorfactory.SupplyLimit(40);
    leadConfig.talonFXConfig.statorCurrLimit = motorfactory.CurrentLimit(80);

    // set closed loop parameters
    setDiffVelocityPidConfigs(cfg, leadConfig.talonFXConfig);

    setMotionMagicConfigs(cfg, leadConfig.talonFXConfig);
    // create motors
    TalonFX leadMotor = motorfactory.createTalonFX(leadID, leadConfig);
    TalonFX followMotor = motorfactory.createTalonFX(followID, followConfig, leadMotor);

    // make sure they use pid
    leadMotor.selectProfileSlot(Constants.PIDSlotVelocity, Constants.PIDLoopPrimary);
    // reset odometry to 0
    leadMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.CANTimeout);

    // send it up
    TalonFX[] motors = new TalonFX[2];
    motors[0] = leadMotor;
    motors[1] = followMotor;
    return motors;
  }

  public void rightInvert(Config.TankDriveConfig cfg, motorfactory.MotorConfiguration motorConfig) {
    TalonFXInvertType invertType;
    if (cfg.physicalLayout.rightInvertIsCounterClockwise) {
      invertType = TalonFXInvertType.CounterClockwise;
    } else {
      invertType = TalonFXInvertType.Clockwise;
    }
    motorConfig.invertType = invertType;
  }

  public void leftInvert(Config.TankDriveConfig cfg, motorfactory.MotorConfiguration motorConfig) {
    TalonFXInvertType invertType;
    if (cfg.physicalLayout.rightInvertIsCounterClockwise) {
      invertType = TalonFXInvertType.Clockwise;
    } else {
      invertType = TalonFXInvertType.CounterClockwise;
    }
    motorConfig.invertType = invertType;
  }

  public void setMotionMagicConfigs(Config.TankDriveConfig cfg, TalonFXConfiguration talonConfig) {
    talonConfig.motionCruiseVelocity =
        (cfg.controlParameters.velocity * 12)
            / TankDriveMath.distancePerTick(
                cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter);
    talonConfig.motionAcceleration =
        talonConfig.motionCruiseVelocity / cfg.controlParameters.accelerationTime;
  }

  public void setDiffVelocityPidConfigs(
      Config.TankDriveConfig cfg, TalonFXConfiguration talonConfig) {
    talonConfig.slot0.kF =
        TankDriveMath.kF(
            cfg.physicalLayout.gearRatio,
            cfg.physicalLayout.wheelDiameter,
            cfg.physicalLayout.predictedVelocity * 12);
    talonConfig.slot0.kP =
        TankDriveMath.kP_velocity(
            cfg.physicalLayout.gearRatio,
            cfg.physicalLayout.wheelDiameter,
            cfg.controlParameters.pSaturation * 12);
  }

  public void setTurnPidConfigs(Config.TankDriveConfig cfg, TalonFXConfiguration talonConfig) {
    talonConfig.slot1.kF = 0;
    // we are setting turn up to be in units of 0.1 degrees
    talonConfig.slot1.kP = TankDriveMath.kP(cfg.controlParameters.turnPSaturation * 10);
  }

  public void setDiffSensors(
      Config.TankDriveConfig cfg,
      TalonFXConfiguration rightConfig,
      TalonFXConfiguration leftConfig) {
    // Set feedback sensor config
    leftConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // rightConfig.remoteFilter0.remoteSensorDeviceID = cfg.TankDrive.leftLeaderID;
    // rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

  }
  /**
   * Determines if SensorSum or SensorDiff should be used for combining left/right sensors into
   * Robot Distance.
   *
   * <p>Assumes Aux Position is set as Remote Sensor 0.
   *
   * <p>configAllSettings must still be called on the master config after this function modifies the
   * config values.
   *
   * @param masterInvertType Invert of the Master Talon
   * @param masterConfig Configuration object to fill
   */
  void setRobotDistanceConfigs(
      TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
    /**
     * Determine if we need a Sum or Difference.
     *
     * <p>The auxiliary Talon FX will always be positive in the forward direction because it's a
     * selected sensor over the CAN bus.
     *
     * <p>The master's native integrated sensor may not always be positive when forward because
     * sensor phase is only applied to *Selected Sensors*, not native sensor sources. And we need
     * the native to be combined with the aux (other side's) distance into a single robot distance.
     */

    /* THIS FUNCTION should not need to be modified.
    This setup will work regardless of whether the master
    is on the Right or Left side since it only deals with
    distance magnitude.  */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise) {
      /*
      	If master is inverted, that means the integrated sensor
      	will be negative in the forward direction.
      	If master is inverted, the final sum/diff result will also be inverted.
      	This is how Talon FX corrects the sensor phase when inverting
      	the motor direction.  This inversion applies to the *Selected Sensor*,
      	not the native value.
      	Will a sensor sum or difference give us a positive total magnitude?
      	Remember the Master is one side of your drivetrain distance and
      	Auxiliary is the other side's distance.
      		Phase | Term 0   |   Term 1  | Result
      	Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
      	Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
      	Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
      */

      masterConfig.diff0Term =
          TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated Sensor
      masterConfig.diff1Term =
          TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      masterConfig.primaryPID.selectedFeedbackSensor =
          TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Diff0 - Diff1
    } else {
      /* Master is not inverted, both sides are positive so we can sum them. */
      masterConfig.sum0Term =
          TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      masterConfig.sum1Term =
          TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
      masterConfig.primaryPID.selectedFeedbackSensor =
          TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0 + Sum1
    }

    /* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
    the real-world value */
    masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
  }

  /**
   * Determines if SensorSum or SensorDiff should be used for combining left/right sensors into
   * Robot Distance.
   *
   * <p>Assumes Aux Position is set as Remote Sensor 0.
   *
   * <p>configAllSettings must still be called on the master config after this function modifies the
   * config values.
   *
   * @param masterInvertType Invert of the Master Talon
   * @param masterConfig Configuration object to fill
   */
  void setRobotTurnConfigs(
      TalonFXInvertType masterInvertType,
      TalonFXConfiguration masterConfig,
      int ticksPerRobotSpin) {
    /**
     * Determine if we need a Sum or Difference.
     *
     * <p>The auxiliary Talon FX will always be positive in the forward direction because it's a
     * selected sensor over the CAN bus.
     *
     * <p>The master's native integrated sensor may not always be positive when forward because
     * sensor phase is only applied to *Selected Sensors*, not native sensor sources. And we need
     * the native to be combined with the aux (other side's) distance into a single robot heading.
     */

    /* THIS FUNCTION should not need to be modified.
    This setup will work regardless of whether the master
    is on the Right or Left side since it only deals with
    heading magnitude.  */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise) {
      /*
      	If master is inverted, that means the integrated sensor
      	will be negative in the forward direction.
      	If master is inverted, the final sum/diff result will also be inverted.
      	This is how Talon FX corrects the sensor phase when inverting
      	the motor direction.  This inversion applies to the *Selected Sensor*,
      	not the native value.
      	Will a sensor sum or difference give us a positive heading?
      	Remember the Master is one side of your drivetrain distance and
      	Auxiliary is the other side's distance.
      		Phase | Term 0   |   Term 1  | Result
      	Sum:  -1 *((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
      	Diff: -1 *((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
      	Diff: -1 *((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
      */

      masterConfig.sum0Term =
          TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated Sensor
      masterConfig.sum1Term =
          TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      masterConfig.auxiliaryPID.selectedFeedbackSensor =
          TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0 + Sum1

      /*
      	PID Polarity
      	With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
      	This is important because if the PID polarity is incorrect, we will run away while trying to correct
      	Will inverting the polarity give us a positive counterclockwise heading?
      	If we're moving counterclockwise(+), and the master is on the right side and inverted,
      	it will have a negative velocity and the auxiliary will have a negative velocity
      	 heading = right + left
      	 heading = (-) + (-)
      	 heading = (-)
      	Let's assume a setpoint of 0 heading.
      	This produces a positive error, in order to cancel the error, the right master needs to
      	drive backwards. This means the PID polarity needs to be inverted to handle this

      	Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
      	it will have a positive velocity and the auxiliary will have a positive velocity.
      	 heading = right + left
      	 heading = (+) + (+)
      	 heading = (+)
      	Let's assume a setpoint of 0 heading.
      	This produces a negative error, in order to cancel the error, the left master needs to
      	drive forwards. This means the PID polarity needs to be inverted to handle this
      */
      masterConfig.auxPIDPolarity = true;
    } else {
      /* Master is not inverted, both sides are positive so we can diff them. */
      masterConfig.diff0Term =
          TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice(); // Aux Selected Sensor
      masterConfig.diff1Term =
          TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
      masterConfig.auxiliaryPID.selectedFeedbackSensor =
          TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Sum0 + Sum1
      /* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
      masterConfig.auxPIDPolarity = true;
    }
    /**
     * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations... -
     * Target param for aux PID1 is 18bits with a range of [-131072,+131072] units. - Target for aux
     * PID1 in motion profile is 14bits with a range of [-8192,+8192] units. ... so at 3600 units
     * per 360', that ensures 0.1 degree precision in firmware closed-loop and motion profile
     * trajectory points can range +-2 rotations.
     */
    masterConfig.auxiliaryPID.selectedFeedbackCoefficient =
        Constants.TurnPIDRange / ticksPerRobotSpin;
  }

  void switchToPigeon() {

    driveRightLead.configAuxPIDPolarity(false, Constants.CANTimeout);
    driveRightLead.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice(),
        Constants.PIDLoopTurn,
        Constants.CANTimeout);
    driveRightLead.configSelectedFeedbackCoefficient(
        Constants.TurnPIDRange / Constants.PigeonUnitsPerRotation,
        Constants.PIDLoopTurn,
        Constants.CANTimeout);
  }

  void switchToEncoder(Config.TankDriveConfig cfg) {
    // TalonFXInvertType.Clockwise is same as setInverted(true)
    driveRightLead.configAuxPIDPolarity(true, Constants.CANTimeout);
    if (cfg.physicalLayout.rightInvertIsCounterClockwise) {
      driveRightLead.configSelectedFeedbackSensor(
          TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(),
          Constants.PIDLoopTurn,
          Constants.CANTimeout);
    } else {
      driveRightLead.configSelectedFeedbackSensor(
          TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(),
          Constants.PIDLoopTurn,
          Constants.CANTimeout);
    }
    double ticksPerRobotSpin =
        TankDriveMath.ticksPerRobotSpin(
            cfg.physicalLayout.wheelBase,
            cfg.physicalLayout.wheelDiameter,
            cfg.physicalLayout.gearRatio);
    driveRightLead.configSelectedFeedbackCoefficient(
        Constants.TurnPIDRange / ticksPerRobotSpin, Constants.PIDLoopTurn, Constants.CANTimeout);
  }
}
