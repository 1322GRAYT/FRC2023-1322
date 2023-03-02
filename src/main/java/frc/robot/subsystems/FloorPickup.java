package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorPickup extends SubsystemBase {

  // Resources
  private WPI_TalonFX FloorMotor = new WPI_TalonFX(Constants.FLOOR_MOTOR);
  private DoubleSolenoid GrabberLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_FLOOR_GRAB_L_0, Constants.PNEUMATIC_FLOOR_GRAB_L_1);
  private DoubleSolenoid GrabberRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_FLOOR_GRAB_R_0, Constants.PNEUMATIC_FLOOR_GRAB_R_1);
  private DoubleSolenoid ConeGrabber = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_LIFT_CONE_GRAB_0, Constants.PNEUMATIC_LIFT_CONE_GRAB_1);
  private WPI_TalonSRX ConeMotor = new WPI_TalonSRX(Constants.CONE_MTR_LIFT);
  private PWM ConeRotateServo = new PWM(Constants.CONE_ROTATE_SERVO);

  // Internal Definitions
  public enum GrabState {
    Open, LeftClose, RightClose, Closed;
  };

  GrabState grabState = GrabState.Closed;
  GrabState coneState = GrabState.Open;

  double floorMotorPower = 0;
  ControlMode floorMotorMode = ControlMode.PercentOutput;

  double coneMotorPower = 0;
  ControlMode coneMotorMode = ControlMode.PercentOutput;

  double coneRotateMotorPower = 0;

  /**
   * Constructor
   */
  public FloorPickup() {

    grabState = GrabState.Open;

    ConeMotor.setInverted(true);

    // Floor Pickup Motor Configuration
    FloorMotor.configFactoryDefault();
    FloorMotor.setSensorPhase(false);
    FloorMotor.setInverted(false);
    FloorMotor.setNeutralMode(NeutralMode.Brake);
  }

  // Getter Interfaces

  public GrabState getGrabState() {
    return grabState;
  }

  public double getFloorMotorPower() {
    return floorMotorPower;
  }

  // Setter Interfaces

  public void setGrabState(GrabState grabState) {
    this.grabState = grabState;
  }

  public void setFloorMotorPower(double floorMotorPower) {
    floorMotorMode = ControlMode.PercentOutput;
    this.floorMotorPower = floorMotorPower;
  }

  public void setConeMotorPower(double coneMotorPower) {
    this.coneMotorPower = coneMotorPower;
  }

  public void setConeRotatePower(double coneRotateMotorPower) {
    this.coneRotateMotorPower = coneRotateMotorPower;
  }

  public void setConeGrab(boolean open) {
    coneState = open ? GrabState.Open : GrabState.Closed;
  }

  // Control Cycles

  private void ControlFloorMotor() {
    FloorMotor.set(floorMotorMode, floorMotorPower);
  }

  private void ControlGrabber() {
    Value setL = Value.kOff;
    Value setR = Value.kOff;

    switch (grabState) {
      case Closed:
        setL = Value.kReverse;
        setR = Value.kReverse;
        break;
      case Open:
        setL = Value.kForward;
        setR = Value.kForward;
        break;
      case LeftClose:
        setL = Value.kReverse;
        setR = Value.kForward;
        break;
      case RightClose:
        setL = Value.kForward;
        setR = Value.kReverse;
        break;
      default:
        break;
    }

    GrabberLeft.set(setL);
    GrabberRight.set(setR);
  }

  private void ControlConeLift() {
    ConeMotor.set(coneMotorMode, coneMotorPower);
  }

  private void ControlConeRotate() {
    ConeRotateServo.setSpeed(coneRotateMotorPower);
  }

  private void ControlConeGrab() {
    Value set = Value.kOff;
    switch (coneState) {
      case Open:
        set = Value.kForward;
        break;
      case Closed:
        set = Value.kReverse;
      default:
        break;
    }
    ConeGrabber.set(set);
  }

  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ControlFloorMotor();
    ControlGrabber();
    ControlConeLift();
    ControlConeRotate();
    ControlConeGrab();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
