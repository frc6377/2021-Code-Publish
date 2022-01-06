package frc.robot.config;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonProperty;

public class Config extends Configable {
  @JsonProperty("tank_drive")
  public TankDriveConfig TankDrive = new TankDriveConfig();

  @JsonProperty("driver_joystick")
  public DriverConfig Driver = new DriverConfig();

  @JsonProperty("gunner_joystick")
  public GunnerConfig Gunner = new GunnerConfig();

  public class DriverConfig {
    @JsonProperty("joystick_port_id")
    public int joystickPortID = 0;

    @JsonProperty("trigger_sensitivity")
    public double triggerSensitivity = 0.2;
  }

  public class GunnerConfig {
    @JsonProperty("joystick_port_id")
    public int joystickPortID = 1;

    @JsonProperty("trigger_sensitivity")
    public double triggerSensitivity = 0.2;
  }

  public class TankDriveConfig {
    public boolean simulated = false;

    @JsonProperty("left_leader_id")
    public int leftLeaderID = 5;

    @JsonProperty("right_leader_id")
    public int rightLeaderID = 4;

    @JsonProperty("left_follow_id")
    public int leftFollowerID = 3;

    @JsonProperty("right_follow_id")
    public int rightFollowerID = 2;

    @JsonProperty("pidgeon_id")
    public int pigeonID = 1;

    @JsonProperty("physical_layout")
    public PhysicalLayout physicalLayout = new PhysicalLayout();

    public class PhysicalLayout {
      @JsonProperty("gear_ratio")
      public double gearRatio = 50 / 11;

      @JsonProperty("wheel_diameter_inch")
      public double wheelDiameter = 4;

      @JsonProperty("wheel_base_inch")
      public double wheelBase = 22.5;

      @JsonProperty("predicted_velocity_feet_per_second")
      public double predictedVelocity = 19.8;

      @JsonProperty("right_side_motor_is_counter_clockwise_forward")
      public boolean rightInvertIsCounterClockwise = true;
    }

    @JsonProperty("control")
    public ControlParameters controlParameters = new ControlParameters();

    public class ControlParameters {
      @JsonProperty("velocity_feet_per_second")
      public double velocity = 17;

      @JsonProperty("error_saturation_point_feet_per_second")
      public double pSaturation = 10;

      @JsonProperty("time_to_max_velocity_seconds")
      public double accelerationTime = 1;

      @JsonProperty("method")
      public Method method = Method.VELOCITY; // 0 is velocity, 1 is differential

      @JsonProperty("turning_saturation_point_degrees")
      public double turnPSaturation = 360;
    }
  }

  @JsonFormat(shape = JsonFormat.Shape.NUMBER_INT)
  public enum Method {
    VELOCITY,
    DIFFERENTIAL
  }
}
