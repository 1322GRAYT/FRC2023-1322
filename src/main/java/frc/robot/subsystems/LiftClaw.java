// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftClaw extends SubsystemBase {

  // Internal Definitions
  public enum ClawState {
    Open,
    Closed
  }

  // Resource Definitions
  private Servo clawRotate = new Servo(Constants.CLAW_ROTATE_SERVO);
  private ClawState clawState;

  private DoubleSolenoid clawGrabSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_CLAW_0, Constants.PNEUMATIC_CLAW_1);
  private AnalogInput clawAngle = new AnalogInput(Constants.CLAW_ANGLE_SENSOR);

  /** Creates a new Claw. */
  public LiftClaw() {}

  public void  clawToggle() {
    if (clawState == ClawState.Open) closeClaw();
    if (clawState == ClawState.Closed) openClaw();
  }
  
  public void closeClaw() {
    clawGrabSolenoid.set(Value.kReverse);
  }

  public void openClaw() {
    clawGrabSolenoid.set(Value.kForward);
  }

  public void rotateClaw(double power) {
    double scale_Factor = 10;
    double currentPosition = clawRotate.get(); // this is where it is...
    double endPosition = 0;
    endPosition = currentPosition + (power * scale_Factor);
    endPosition = (endPosition > 127) ? 127 : endPosition;
    endPosition = (endPosition < -127) ? -127 : endPosition;
    clawRotate.set(endPosition);
  }

  public double getClawPosition() {
    return clawRotate.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
