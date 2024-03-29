package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {
  private TalonFX _shooterMotor0;
  private TalonFX _shooterMotor1;
  private TalonFX _preshootMotor;

  private DigitalInput _shooterSensor0, _shooterSensor1, _shooterSensor2;

                 

  public ShooterSubsystem(int SHOOTER_MOTOR_0, 
                int SHOOTER_MOTOR_1, 
                int SHOOTER_PRESHOOT,
                int SHOOTER_SENSOR_0,
                int SHOOTER_SENSOR_1,
                int SHOOTER_SENSOR_2) {
    _shooterMotor0 = new TalonFX(SHOOTER_MOTOR_0);
    configMotor(_shooterMotor0);
    _shooterMotor1 = new TalonFX(SHOOTER_MOTOR_1);
    configMotor(_shooterMotor1);
    _preshootMotor = new TalonFX(SHOOTER_PRESHOOT);
    configMotor(_preshootMotor);
    _shooterSensor0 = new DigitalInput(SHOOTER_SENSOR_0);
    _shooterSensor1 = new DigitalInput(SHOOTER_SENSOR_1);
    _shooterSensor2 = new DigitalInput(SHOOTER_SENSOR_2);
  }

  private void configMotor(TalonFX motor) {
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
    //motor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
    motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
}

public void move(double speed) {
  _shooterMotor0.set(ControlMode.PercentOutput, speed);
  _shooterMotor1.set(ControlMode.PercentOutput, -speed);
}

public void preShootMove(double speed) {
  _preshootMotor.set(ControlMode.PercentOutput, speed);
}
  public void shoot() {
    _shooterMotor0.set(ControlMode.PercentOutput, 1.0);
    _shooterMotor1.set(ControlMode.PercentOutput, -1.0);
  }

  static enum state {
    UNLOADED, LOADED, TOOFAR
  }

  public void loadRing() {
    double velocity = Constants.PRESHOOT_INITIAL_VELOCITY;
    // preshoot load until it is in the right position

    Timer timer = new Timer();
    timer.start();

    state currentState = state.UNLOADED;

    boolean done = false;
    while (timer.get() < Constants.PRESHOOT_TIME && !done) {
      _preshootMotor.set(ControlMode.PercentOutput, velocity);
      
      switch (currentState) {
        case UNLOADED:
          if (_shooterSensor0.get()) {
            velocity *= 0.5;
            currentState = state.LOADED;
          }
          break;
        case LOADED:
          if (_shooterSensor1.get()) {
            velocity *= 0;
            _preshootMotor.set(ControlMode.PercentOutput, 0);
            done = true;
          }
          if (_shooterSensor2.get()) {
            velocity *= -0.5;
            currentState = state.TOOFAR;
          }
          break;
        case TOOFAR:
          if (_shooterSensor1.get()) {
            velocity *= 0;
            _preshootMotor.set(ControlMode.PercentOutput, 0);
            done = true;
          }
          if (_shooterSensor0.get()) {
            velocity *= -0.5;
            currentState = state.LOADED;
          }
          break;
      }
     }
  }


}