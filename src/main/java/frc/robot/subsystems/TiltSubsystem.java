package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TiltSubsystem extends SubsystemBase {
  private TalonFX _tiltMotor;
  public TiltSubsystem(int TILT_MOTOR) {
    _tiltMotor = new TalonFX(TILT_MOTOR);
    Constants.configMotor(_tiltMotor);
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

/* private void configMotor(TalonFX motor) {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    /* Swerve Drive Motor Configuration * /
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
        Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
        Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
        Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

    talonFXConfig.slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
    talonFXConfig.slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
    talonFXConfig.slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
    talonFXConfig.slot0.kF = Constants.SwerveDrivetrain.DRIVE_kF;        
    talonFXConfig.supplyCurrLimit = driveSupplyLimit;
    talonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    talonFXConfig.openloopRamp = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
    talonFXConfig.closedloopRamp = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;
    
    motor.configFactoryDefault();
    motor.configAllSettings(talonFXConfig);
    //motor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
    motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
}*/
}