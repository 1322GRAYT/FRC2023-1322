package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import frc.robot.Constants;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX _shooterMotor0;
  private TalonFX _shooterMotor1;
  private TalonFX _preshootMotor;

  private DigitalInput _shooterSensor0, _shooterSensor1, _shooterSensor2;

  private boolean shooting = false;
  private boolean loading = false;

  public ShooterSubsystem(int SHOOTER_MOTOR_0,
      int SHOOTER_MOTOR_1,
      int SHOOTER_PRESHOOT,
      int SHOOTER_SENSOR_0,
      int SHOOTER_SENSOR_1,
      int SHOOTER_SENSOR_2) {
    _shooterMotor0 = new TalonFX(SHOOTER_MOTOR_0);
    configMotor(_shooterMotor0, false);
    _shooterMotor1 = new TalonFX(SHOOTER_MOTOR_1);
    configMotor(_shooterMotor1, true);
    _preshootMotor = new TalonFX(SHOOTER_PRESHOOT);
    configMotor(_preshootMotor, false);
    _shooterSensor0 = new DigitalInput(SHOOTER_SENSOR_0);
    _shooterSensor1 = new DigitalInput(SHOOTER_SENSOR_1);
    _shooterSensor2 = new DigitalInput(SHOOTER_SENSOR_2);
  }

  private void configMotor(TalonFX motor, boolean inverted) {
    TalonFXConfiguration swerveDriveTalonFXConfig = new TalonFXConfiguration();

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT,
        Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL,
        Constants.SwerveDrivetrain.DRIVE_PEAK_CL,
        Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

    swerveDriveTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
    swerveDriveTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
    swerveDriveTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
    swerveDriveTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.DRIVE_kF;
    swerveDriveTalonFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveDriveTalonFXConfig.openloopRamp = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
    swerveDriveTalonFXConfig.closedloopRamp = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;

    motor.configFactoryDefault();
    motor.configAllSettings(swerveDriveTalonFXConfig);
    motor.setInverted((inverted) ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
    motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
  }

  public void move(double speed) {
    _shooterMotor0.set(ControlMode.PercentOutput, speed);
    _shooterMotor1.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    _shooterMotor0.set(ControlMode.PercentOutput, 0.0);
    _shooterMotor1.set(ControlMode.PercentOutput, 0.0);
  }

  public void preShootMove(double speed) {
    _preshootMotor.set(ControlMode.PercentOutput, speed);
  }

  public void preShootStop() {
    _preshootMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void drop() {
    shooting = true;
    try {
    _preshootMotor.set(ControlMode.PercentOutput,1.0);
    Thread.sleep(Constants.SHOOTER_DWELL_TIME);
    }
    catch (InterruptedException ignored) {

    }
    finally {
      _preshootMotor.set(ControlMode.PercentOutput, 0);
      shooting=false;
    }
  }

  public void shoot() {
    shooting = true;
    try {
      move(-Constants.SHOOTER_VELOCITY);
      Thread.sleep(Constants.SHOOTER_DWELL_TIME);
      preShootMove(-Constants.PRESHOOT_SHOOT_SPEED);
      Thread.sleep(Constants.SHOOTER_DWELL_TIME);
    } catch (InterruptedException ignored) {

    } finally {
      preShootStop();
      stop();
      shooting = false;
    }
  }

  public void setLoading() {
    loading = true;
  }

  public void unSetLoading() {
    loading = false;
  }

  public boolean loaded() {
    return !_shooterSensor1.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Sensor 0", _shooterSensor0.get() ? "Detected" : "Not Detected");
    SmartDashboard.putString("Sensor 1", _shooterSensor1.get() ? "Detected" : "Not Detected");
    SmartDashboard.putString("Sensor 2", _shooterSensor2.get() ? "Detected" : "Not Detected");

    if (!shooting && !loading) {
      if (_shooterSensor2.get() == false) {
        do {
          preShootMove(Constants.PRESHOOT_ADJUST_SPEED);
        } while (_shooterSensor1.get() == true);
        preShootMove(0.0);
      } else if (_shooterSensor0.get() == false) {
        do {
          preShootMove(-Constants.PRESHOOT_ADJUST_SPEED);
        } while (_shooterSensor1.get() == true);
        preShootMove(0.0);
      } else
        preShootMove(0.0);
    }
  }
}