package frc.robot.motorMath;

public class PID {
  public PID(double p, double f) {
    this(p, 0.0, 0.0, f);
  }

  public PID(double p, double i, double d, double f) {
    kP = p;
    kI = i;
    kD = d;
    kF = f;
  }

  public double kP = 0.0;
  public double kI = 0.0;
  public double kD = 0.0;
  public double kF = 0.0;
}
