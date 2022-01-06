package frc.robot.config;

/** Exception class used when we get an error attempting to create a config class. */
public class ConfigCreationException extends Exception {
  private static final long serialVersionUID = 1L;

  public ConfigCreationException(String errorMessage) {
    super(errorMessage);
  }

  public ConfigCreationException(String errorMessage, java.lang.Throwable cause) {
    super(errorMessage, cause);
  }
}
