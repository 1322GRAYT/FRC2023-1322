package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_LIFT;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmClawSubsystem extends SubsystemBase {

  public enum controlState {
    Back,
    Mid,
    Front
  }

  public enum ClawState {
    Open,
    Closed
  }

  private WPI_TalonFX ArmMotor = new WPI_TalonFX(Constants.ARM_MOTOR, "rio");
  private WPI_TalonFX ArmRotateMotor = new WPI_TalonFX(Constants.ARM_ROTATE_MOTOR, "rio");

  private Solenoid[] LeftArmSolenoids = {
      new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_ARM_TILT_LEFT_0),
      new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_ARM_TILT_LEFT_1)
  };
  private Solenoid[] RightArmSolenoids = {
      new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_ARM_TILT_RIGHT_0),
      new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_ARM_TILT_RIGHT_1)
  };

  private Solenoid clawGrabSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_CLAW);
  // private AnalogInput clawAngle = new AnalogInput(CLAW_ANGLE_SENSOR);

  private Servo clawRotate = new Servo(Constants.CLAW_ROTATE_SERVO);

  private DigitalInput TrackMidTrig = new DigitalInput(Constants.SW_LIFT_TRACK_TRIG);

  private controlState armControlState;

  private ClawState clawState;

  /********************************/
  /* LiftSubsystem Constructor */
  /********************************/
  public ArmClawSubsystem() {

    /*****************************************************************/
    /* Lift Motor Controller Configurations */
    /*****************************************************************/
    ArmMotor.configFactoryDefault();
    ArmMotor.setSensorPhase(false);
    ArmMotor.setInverted(false);
    ArmMotor.setNeutralMode(NeutralMode.Brake);

    ArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, (int) 0, (int) 0);
    ArmMotor.setSelectedSensorPosition(0);
    ArmMotor.config_kP(0, K_LIFT.KeLIFT_K_Prop);
    ArmMotor.config_kI(0, K_LIFT.KeLIFT_K_Intgl);
    ArmMotor.config_kD(0, K_LIFT.KeLIFT_K_Deriv);
    ArmMotor.config_IntegralZone(0, K_LIFT.KeLIFT_r_IntglErrMaxEnbl);
    ArmMotor.config_kF(0, K_LIFT.KeLIFT_K_FdFwd);

    ArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    ArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    armControlState = controlState.Back;

  }

  public void  clawToggle() {
    if (clawState == ClawState.Open) closeClaw();
    if (clawState == ClawState.Closed) openClaw();
  }
  
  public void closeClaw() {
    clawGrabSolenoid.set(false);
  }

  public controlState getArmControlState() {
    return armControlState;
  }

  public boolean detectTrackMidTrigger() {
    return (TrackMidTrig.get() == false);
  }

  public boolean detectTrackLimitFront() {
    return (ArmMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1);
  }

  public boolean detectTrackLimitRear() {
    return (ArmMotor.getSensorCollection().isRevLimitSwitchClosed() == 1);
  }

  public controlState getControlState() {
    return armControlState;
  }

  public void halt() {
    getArmMotor().set(ControlMode.PercentOutput, 0);
  }

  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
    setControlState(controlState.Back);
  }

  public WPI_TalonFX getArmMotor() {
    return ArmMotor;
  }

  public double getPosition() {
    return getArmMotor().getSelectedSensorPosition();
  }

  public void openClaw() {
    clawGrabSolenoid.set(true);
  }

  public void rotateClaw(double power) {
    double scale_Factor = 10;
    double currentPosition = clawRotate.get(); // this is where it is...
    double endPosition = 0;
    endPosition = currentPosition + (power * scale_Factor);
    endPosition = (endPosition > 127) ? 127 : endPosition;
    endPosition = (endPosition < -127) ? -127 : endPosition;
    clawRotate.set(endPosition);
  }

  public double getClawPosition() {
    return clawRotate.get();
  }

  public void runArmAtSpeed(double speed) {
    getArmMotor().set(ControlMode.Velocity, speed);
  }

  public void runArmAtPower(double pwr) {
    getArmMotor().set(ControlMode.PercentOutput, pwr);
  }

  public void setArmControlState(controlState armControlState) {
    this.armControlState = armControlState;
  }

  public void setArmPosition() {
    switch (armControlState) {
      case Back:
        LeftArmSolenoids[0].set(false);
        RightArmSolenoids[0].set(false);
        LeftArmSolenoids[1].set(false);
        RightArmSolenoids[1].set(false);
        break;
      case Mid:
        LeftArmSolenoids[0].set(true);
        RightArmSolenoids[0].set(true);
        LeftArmSolenoids[1].set(false);
        RightArmSolenoids[1].set(false);
        break;
      case Front:
        LeftArmSolenoids[0].set(true);
        RightArmSolenoids[0].set(true);
        LeftArmSolenoids[1].set(true);
        RightArmSolenoids[1].set(true);
        break;
      default:
        break;
    }
  }

  public void setPositionPID(boolean activate, double pstn) {
    if (activate == true) {
      getArmMotor().set(ControlMode.Position, pstn);
    } else {
      halt();
    }
  }

  public void setControlState(controlState controlSt) {
    armControlState = controlSt;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
