package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  // private WPI_TalonFX _liftMotor;
  private CANSparkMax _liftMotor;
  private RelativeEncoder _liftEncoder;
  private PIDController _pid;

  public LiftSubsystem(int lift_motor_id) {
    _liftMotor = new CANSparkMax(lift_motor_id, MotorType.kBrushless);
    _liftMotor.setIdleMode(IdleMode.kBrake);
    _liftMotor.setSmartCurrentLimit(Constants.LIFT_CURRENT_LIMIT);
    _liftEncoder = _liftMotor.getEncoder();
    _pid = new PIDController(
        Constants.LIFT_KP,
        Constants.LIFT_KI,
        Constants.LIFT_KD);
  }


  public void setHomePosition() {
    _liftMotor.set(1);
    //wait for .5 seconds
    Timer.delay(0.5);
    _liftEncoder.setPosition(0);
  }

  public void setPositionWait(double position) {
    _pid.setSetpoint(position);
    while (Math.abs(_liftEncoder.getPosition() - position) > 15) {
      _liftMotor.set(_pid.calculate(_liftEncoder.getPosition()));
    }
  }

  public void setPositionNoWait(double position) {
    _pid.setSetpoint(position);
    new Thread() {
      public void run() {
        while (Math.abs(_liftEncoder.getPosition() - position) > 15) {
          _liftMotor.set(_pid.calculate(_liftEncoder.getPosition()));
        }
      }
    }.start();
  }

  public void loadPosition() {
    down(1.0);
  }

  public void up(double speed) {
    _liftMotor.set(speed);
  }

  public void down(double speed) {
    _liftMotor.set(-speed);
  }

  public void move(double speed) {
    _liftMotor.set(speed);
  }
}