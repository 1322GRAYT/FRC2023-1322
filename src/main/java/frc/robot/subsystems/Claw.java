package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Claw extends SubsystemBase {

  // Internal Definitions
  public enum ClawState {
    Open, Closed;
  }

  private ClawState clawState = ClawState.Closed;
  double intakeMotorPower = 0;

  ControlMode intakeMotorMode = ControlMode.PercentOutput;

  private DoubleSolenoid clawGrab = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_CLAW_0,
      Constants.PNEUMATIC_CLAW_1);
  private DigitalInput clawGrabSensor = new DigitalInput(5);

  public Claw() {
  }

  public boolean getClawGrabSensor() {
    return clawGrabSensor.get();
  }

  public void setIntakeMotorPower(double coneMotorPower) {
    intakeMotorPower = coneMotorPower;
  }

  public ClawState getClawState() {
    return clawState;
  }

  public boolean getGrabSensor() {
    return clawGrabSensor.get();
  }

  public void setClawState(ClawState clawState) {
    this.clawState = clawState;
  }

  private void ControlClaw() {
    Value set = Value.kOff;
    switch (clawState) {
      case Closed:
        set = Value.kReverse;
        break;
      case Open:
        set = Value.kForward;
        break;
    }

    clawGrab.set(set);
  }

  public void init_periodic() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ControlClaw();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
