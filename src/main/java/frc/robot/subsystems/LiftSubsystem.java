package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  //private WPI_TalonFX _liftMotor;
  private CANSparkMax _liftMotor;
  public LiftSubsystem(int lift_motor_id) {
    _liftMotor = new CANSparkMax(lift_motor_id, MotorType.kBrushless);  
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