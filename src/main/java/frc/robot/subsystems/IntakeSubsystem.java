package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax _intakeMotor;
  public IntakeSubsystem(int INTAKE_MOTOR) {
    _intakeMotor = new CANSparkMax(INTAKE_MOTOR,MotorType.kBrushless);
  }

  public void intakeOn() {
    intake(1.0);
  }
  public void intakeOff() {
    intake(0.0);
  }
  
  public void intake(double speed) {
    _intakeMotor.set(speed);
  }

  public void discharge(double speed) {
    _intakeMotor.set(-speed);
  }

  public void move(double speed) {
    _intakeMotor.set(speed);
  }
}