package config;

import com.fasterxml.jackson.annotation.JsonProperty;
import frc.robot.config.Configable;

public class TestConfig extends Configable {
  @JsonProperty("config_a")
  public String ConfigA = "test1";

  @JsonProperty("config_b")
  public String ConfigB = "test2";

  public TestConfig() {}
}
