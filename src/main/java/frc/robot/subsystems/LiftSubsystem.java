package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  //private WPI_TalonFX _liftMotor;
  private TalonFX _liftMotor;
  public LiftSubsystem(int lift_motor_id) {
    _liftMotor = new TalonFX(lift_motor_id);
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
    _liftMotor.configAllSettings(talonFXConfig);

    _liftMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
}


  public void loadPosition() {
    down(1.0);
  }
  
  public void up(double speed) {
    _liftMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void down(double speed) {
    _liftMotor.set(TalonFXControlMode.PercentOutput, -speed);
  }

  public void move(double speed) {
    //System.out.println("speed = " + speed);
    _liftMotor.set(TalonFXControlMode.PercentOutput, speed);
  }
}