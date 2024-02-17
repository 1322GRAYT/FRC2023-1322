package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TiltSubsystem extends SubsystemBase {
  private TalonFX _tiltMotor;
  public TiltSubsystem(int TILT_MOTOR) {
    _tiltMotor = new TalonFX(TILT_MOTOR);
    Constants.configMotor(_tiltMotor,NeutralMode.Brake);
    _tiltMotor.setSelectedSensorPosition(0);
  }

  public void loadPosition() {
    _tiltMotor.set(TalonFXControlMode.Position, Constants.TILT_LOAD_POSITION);
  }

  public void up(double speed) {
    _tiltMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void down(double speed) {
    _tiltMotor.set(TalonFXControlMode.PercentOutput, -speed);
  }

  public void move(double speed) {
    _tiltMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

@Override
public void periodic() {
  SmartDashboard.putNumber("Tilt Position", _tiltMotor.getSelectedSensorPosition());
}
}