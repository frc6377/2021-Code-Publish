package frc.robot.motorMath;

import frc.robot.Constants;

public class TankDriveMath {
  public TankDriveMath() {}

  /**
   * Calculate the distance per motor tick.
   *
   * @param gearRatio The number of times the motor rotates for 1 rotation of the robot wheel.
   * @param wheelDiameter The diameter of the robot wheel. The units @param wheelDiameter is in will
   *     determine the returned units.
   * @param ticksPerMotorRevolution The number of ticks in 1 revolution of the motor. Default: 2048
   * @return The distance per motor tick. Units will be the same as the @param wheelDiameter units.
   */
  public static double distancePerTick(
      double gearRatio, double wheelDiameter, double ticksPerMotorRevolution) {
    return Math.PI * wheelDiameter / (gearRatio * ticksPerMotorRevolution);
  }

  public static double distancePerTick(double gearRatio, double wheelDiameter) {
    return distancePerTick(gearRatio, wheelDiameter, Constants.TalonFXTicksPerRevolution);
  }

  /**
   * Calculate a Feed forward constant for velocity FPID.
   *
   * <p>NOTE: The units of @param wheelDiameter and @param predictedMaxVelocity should match.
   *
   * @param gearRatio The number of times the motor rotates for 1 rotation of the robot wheel.
   * @param wheelDiameter The diameter of the robot wheel.
   * @param predictedMaxVelocity The predicted max velocity of the system. Velocity units should be
   *     the distance units of @param wheelDiameter per second.
   * @param ticksPerMotorRevolution The number of ticks in 1 revolution of the motor. Default: 2048
   * @return The calculated feed forward constant in talon constant units.
   */
  public static double kF(
      double gearRatio,
      double wheelDiameter,
      double predictedMaxVelocity,
      double ticksPerMotorRevolution) {
    double velocityTicksPerSecond =
        predictedMaxVelocity / distancePerTick(gearRatio, wheelDiameter, ticksPerMotorRevolution);
    return kF(velocityTicksPerSecond);
  }

  public static double kF(double gearRatio, double wheelDiameter, double predictedMaxVelocity) {
    return kF(gearRatio, wheelDiameter, predictedMaxVelocity, Constants.TalonFXTicksPerRevolution);
  }

  /**
   * Calculate a Feed forward constant for FPID.
   *
   * @param predictedMaxVelocity The predicted max velocity expressed in ticks per second.
   * @return The calculated feed forward constant in talon constant units.
   */
  public static double kF(double predictedMaxVelocity) {
    double velocityNativeUnits = predictedMaxVelocity / 10;
    return talonConstant(velocityNativeUnits);
  }

  /**
   * Calculate the porportional gain for FPID.
   *
   * @param gearRatio The number of times the motor rotates for 1 rotation of the robot wheel.
   * @param wheelDiameter The diameter of the robot wheel.
   * @param saturation The distance or velocity to saturate at. The units should be the same as the
   *     units of @param wheelDiameter, and if velocity per second.
   * @param ticksPerMotorRevolution The number of ticks in 1 revolution of the motor. Default: 2048
   * @param isVelocity Whether or not the saturation is a velocity or not. Default: True
   * @return The calculated porportional gain constant in talon constant units
   */
  public static double kP(
      double gearRatio,
      double wheelDiameter,
      double saturation,
      double ticksPerMotorRevolution,
      boolean isVelocity) {
    double saturationTicks =
        saturation / distancePerTick(gearRatio, wheelDiameter, ticksPerMotorRevolution);
    if (isVelocity) {
      // convert to native units
      saturationTicks = saturationTicks / 10;
    }
    return kP(saturationTicks);
  }

  public static double kP(
      double gearRatio, double wheelDiameter, double saturation, boolean isVelocity) {
    return kP(
        gearRatio, wheelDiameter, saturation, Constants.TalonFXTicksPerRevolution, isVelocity);
  }

  /**
   * Calculate the porportional gain for FPID.
   *
   * @param saturation The distance (ticks) or velocity (ticks per 0.1 seconds) to saturate at.
   * @return The calculated porportional gain constant in talon constant units
   */
  public static double kP(double saturation) {
    return talonConstant(saturation);
  }

  public static double kP_velocity(
      double gearRatio, double wheelDiameter, double saturation, double ticksPerMotorRevolution) {
    return kP(gearRatio, wheelDiameter, saturation, ticksPerMotorRevolution, true);
  }

  public static double kP_velocity(double gearRatio, double wheelDiameter, double saturation) {
    return kP(gearRatio, wheelDiameter, saturation, true);
  }

  public static double kP_distance(
      double gearRatio, double wheelDiameter, double saturation, double ticksPerMotorRevolution) {
    return kP(gearRatio, wheelDiameter, saturation, ticksPerMotorRevolution, false);
  }

  public static double kP_distance(double gearRatio, double wheelDiameter, double saturation) {
    return kP(gearRatio, wheelDiameter, saturation, false);
  }

  /**
   * Convert a saturation distance (ticks) or velocity (ticks per 100 milliseconds) to a talon
   * constant.
   *
   * @param ticksOrNativeUnits Saturation distance (ticks) or velocity (ticks per 0.1 seconds)
   * @return Talon constant
   */
  public static double talonConstant(double ticksOrNativeUnits) {
    return Constants.TalonFullPowerInternal / ticksOrNativeUnits;
  }

  /**
   * Calculate the number of ticks offset required to cause the robot to yaw 360 degrees. Units
   * of @param wheelBase and @param wheelDiameter must be in the same units.
   *
   * @param wheelBase The distance from the center of a middle left wheel to the center of an edge
   *     right wheel.
   * @param wheelDiameter The diameter of the robot wheel.
   * @param gearRatio The number of times the motor rotates for 1 rotation of the robot wheel.
   * @param ticksPerMotorRevolution The number of ticks in 1 revolution of the motor. Default: 2048
   * @return number of ticks for 1 revolution of the robot.
   */
  public static double ticksPerRobotSpin(
      double wheelBase, double wheelDiameter, double gearRatio, double ticksPerMotorRevolution) {
    return (wheelBase / wheelDiameter) * gearRatio * ticksPerMotorRevolution;
  }

  public static double ticksPerRobotSpin(double wheelBase, double wheelDiameter, double gearRatio) {
    return ticksPerRobotSpin(
        wheelBase, wheelDiameter, gearRatio, Constants.TalonFXTicksPerRevolution);
  }
}
