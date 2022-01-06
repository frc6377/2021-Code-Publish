package frc.robot.subsystems;

public class IntakeSubsystem {
  public void Retract() {
    System.out.println("retract intake");
  }

  public void Deploy() {
    System.out.println("deploy intake");
  }

  public void IntakeCells(boolean intakeOn) {
    System.out.println("intake cells is " + intakeOn);
  }

  public void EjectCells(boolean ejectOn) {
    System.out.println("eject cells is " + ejectOn);
  }
}
