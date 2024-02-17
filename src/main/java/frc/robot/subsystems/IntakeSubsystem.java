package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  private TalonFX _intakeMotor;
  public IntakeSubsystem(int INTAKE_MOTOR) {
    _intakeMotor = new TalonFX(INTAKE_MOTOR);
  }

  public void intakeOn() {
    intake(0.5);
  }
  public void intakeOff() {
    intake(0.0);
  }
  
  public void intake(double speed) {
    _intakeMotor.set(TalonFXControlMode.PercentOutput,speed);
  }

  public void discharge(double speed) {
    _intakeMotor.set(TalonFXControlMode.PercentOutput,-speed);
  }

  public void move(double speed) {
    _intakeMotor.set(TalonFXControlMode.PercentOutput,speed);
  }
}