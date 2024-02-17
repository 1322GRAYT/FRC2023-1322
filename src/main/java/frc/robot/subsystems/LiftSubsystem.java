package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  // private WPI_TalonFX _liftMotor;
  private TalonFX _liftMotor;

  public LiftSubsystem(int lift_motor_id) {
    _liftMotor = new TalonFX(lift_motor_id);
    Constants.configMotor(_liftMotor,NeutralMode.Brake);
    _liftMotor.setSelectedSensorPosition(0);
  }


  public void setHomePosition() {
    //_liftMotor.set(1);
  }

  public void setPositionWait(double position) {
//
  }

  public void setPositionNoWait(double position) {
//
  }

  public void loadPosition() {
    down(1.0);
  }

  public void up(double speed) {
    _liftMotor.set(TalonFXControlMode.PercentOutput,speed);
  }

  public void down(double speed) {
    _liftMotor.set(TalonFXControlMode.PercentOutput,-speed);
  }

  public void move(double speed) {
    _liftMotor.set(TalonFXControlMode.PercentOutput,speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Position", _liftMotor.getSelectedSensorPosition());
  }
}