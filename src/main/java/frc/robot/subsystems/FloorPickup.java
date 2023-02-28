package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorPickup extends SubsystemBase {

  // Resources
  private WPI_TalonFX FloorMotor = new WPI_TalonFX(Constants.LIFT_MOTOR, "rio");
  private DoubleSolenoid GrabberLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_FLOOR_GRAB_L_0, Constants.PNEUMATIC_FLOOR_GRAB_L_1);
  private DoubleSolenoid GrabberRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_FLOOR_GRAB_R_0, Constants.PNEUMATIC_FLOOR_GRAB_R_1);

  // Internal Definitions
  public enum GrabState {
    Open, LeftClose, RightClose, Closed;
  };

  GrabState grabState;

  double floorMotorPower = 0;
  ControlMode floorMotorMode = ControlMode.PercentOutput;

  

  /**
   * Constructor
   */
  public FloorPickup() {

    grabState = GrabState.Open;

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

  // Control Cycles


  private void ControlFloorMotor(){
    FloorMotor.set(floorMotorMode, floorMotorPower);
  }

  private void ControlGrabber(){
    Value setL = Value.kOff;
    Value setR = Value.kOff;

    switch(grabState){
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


  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ControlFloorMotor();
    ControlGrabber();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
