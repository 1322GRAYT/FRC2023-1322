package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX _intakeMotor;
  public IntakeSubsystem(int INTAKE_MOTOR) {
    _intakeMotor = new TalonFX(INTAKE_MOTOR);
    configMotor(_intakeMotor);
  }

  public void intakeOn() {
    intake(1.0);
  }
  public void intakeOff() {
    intake(0.0);
  }
  
  public void intake(double speed) {
    _intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void discharge(double speed) {
    _intakeMotor.set(TalonFXControlMode.PercentOutput, -speed);
  }

  public void move(double speed) {
    _intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  private void configMotor(TalonFX motor) {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    /* Swerve Drive Motor Configuration */
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
}
}