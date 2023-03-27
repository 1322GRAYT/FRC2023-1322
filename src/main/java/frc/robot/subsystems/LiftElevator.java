package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LiftElevator extends SubsystemBase {


  // Internal Definitions
  public enum pitchState {
    Back, Mid, Front;

    private static final pitchState[] vals = values();

    public pitchState next() {
      return vals[(this.ordinal() + 1) % vals.length];
    }

    public pitchState previous() {
      return vals[(this.ordinal() - 1) % vals.length];
    }

  }

  public boolean setPosition = false;

  // TODO: Determine Numbers
  private static final int CUBE_INTAKE_PITCH = 61000;
  private static final int CUBE_INTAKE_ELEVATOR = 196000;
  private static final int CONE_INTAKE_PITCH = 61000;
  private static final int CONE_INTAKE_ELEVATOR = 196000;

  public enum DeliveryState {
    Bottom(1,2,3,4), 
    Mid(1,2,3,4), 
    Top(260000,310800,260000,310800);

    public int CubePointElevator, CubePointPitch, ConePointElevator, ConePointPitch;
    DeliveryState(int CubePointElevator, int CubePointPitch, int ConePointElevator, int ConePointPitch){
      this.CubePointElevator = CubePointElevator;
      this.CubePointPitch = CubePointPitch;
      this.ConePointElevator = ConePointElevator;
      this.ConePointPitch = ConePointPitch;
    }

    private static final DeliveryState[] vals = values();

    public DeliveryState next() {
      return vals[(ordinal() + 1) % vals.length];
    }

    public DeliveryState previous() {
      return vals[(ordinal() - 1) % vals.length];
    }
  }

  public enum Element {
    CUBE, CONE;
  }

  private Element currentElement = Element.CONE;
  private DeliveryState currentDeliverHeight = DeliveryState.Top;

  private double setElevatorInput = 0;
  private ControlMode setElevatorControlMethod = ControlMode.PercentOutput;

  private double setPitchInput = 0;
  private ControlMode setPitchControlMethod = ControlMode.PercentOutput;

  // Resources
  private WPI_TalonFX ElevatorMotor = new WPI_TalonFX(Constants.ELEVATOR_MOTOR);
  private WPI_TalonFX ElevatorPitchMotor = new WPI_TalonFX(Constants.ELEVATOR_PITCH_MOTOR);

  private DigitalInput TrackMidTrig = new DigitalInput(Constants.SW_LIFT_TRACK_TRIG);

  /**
   * Contructor
   */
  public LiftElevator() {
    ElevatorMotor.setInverted(true);
    ElevatorMotor.setNeutralMode(NeutralMode.Brake);
    ElevatorPitchMotor.setInverted(true);
  }

  // Set Interfaces

  public void haltElevator() {
    ElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setPitchPosition(double setPoint) {
    setPitchControlMethod = ControlMode.MotionMagic;
    setPitchInput = setPoint;
  }

  public void setPitchControl(ControlMode mode, double setValue) {
    setPitchControlMethod = mode;
    setPitchInput = setValue;
  }

  public void setPitchPercentPower(double setPower) {
    setPitchControlMethod = ControlMode.PercentOutput;
    setPitchInput = -setPower;
  }

  public void increaseDeliveryHeight() {
    currentDeliverHeight = currentDeliverHeight.next();
  }

  public void decreaseDeliveryHeight() {
    currentDeliverHeight = currentDeliverHeight.previous();
  }

  double topPos;

  public void setTopPos() {
    topPos = ElevatorMotor.getSelectedSensorPosition(0);
  }

  /**
   * Sets control mode to Motion Magic
   * TODO: develop tick conversion factor, approx 5500 ticks per inch
   * 
   * @param setPoint Set point for motion magic, currently set as ticks. 5500
   *                 ticks per inch approx
   */
  public void setElevatorPosition(double setPoint) {
    if (setPoint < 0) {
      setPoint += topPos;
    }
    setPosition = true;
    setElevatorControlMethod = ControlMode.MotionMagic;
    setElevatorInput = setPoint;
  }

  public void setElevatorControl(ControlMode mode, double setValue) {
    
      setElevatorControlMethod = mode;
      setElevatorInput = setValue;
  }

  public void setElevatorPercentPower(double setPower) {
    // if (!setPosition) {
      setElevatorControlMethod = ControlMode.PercentOutput;
      setElevatorInput = -setPower;
    // }
  }

  public void setCurrentElement(Element currentElement) {
    this.currentElement = currentElement;
  }

  public void setGotoDelivery(){
    switch(currentElement){
      case CONE:
        setElevatorInput = currentDeliverHeight.ConePointElevator;
        setPitchInput = currentDeliverHeight.ConePointPitch;
        break;
      case CUBE:
        setElevatorInput = currentDeliverHeight.CubePointElevator;
        setPitchInput = currentDeliverHeight.CubePointPitch;
        break;
    }

    setElevatorControlMethod = ControlMode.MotionMagic;
    setPitchControlMethod = ControlMode.MotionMagic;
  }

  public void setGotoIntake(){
    switch(currentElement){
      case CONE:
        setElevatorInput = CONE_INTAKE_ELEVATOR;
        setPitchInput = CONE_INTAKE_PITCH;
        break;
      case CUBE:
        setElevatorInput = CUBE_INTAKE_ELEVATOR;
        setPitchInput = CUBE_INTAKE_PITCH;
        break;
    }

    setElevatorControlMethod = ControlMode.MotionMagic;
    setPitchControlMethod = ControlMode.MotionMagic;
    
  }

  // Get Interfaces

  /**
   * Gets status of mid Switch
   * 
   * @return State of the Mid elevator switch. A magnetic switch.
   */
  public boolean getMidLimit() {
    return TrackMidTrig.get();
  }

  /**
   * Gets status of top magnetic switch
   * 
   * @return State of top magnetic switch. Top safety limit. Automatically
   *         switches off motor
   */
  public boolean getTopSwitch() {
    return (ElevatorMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1);
  }

  /**
   * Gets status of bottom magnetic switch
   * 
   * @return State of bottom magnetic switch. Bottom safety limit. Automatically
   *         switches off motor and zeroes encoder.
   */
  public boolean getBottomSwitch() {
    return (ElevatorMotor.getSensorCollection().isRevLimitSwitchClosed() == 1);
  }

  public boolean getPitchOutSwitch() {
    return (ElevatorPitchMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1);
  }

  public boolean getPitchInSwitch() {
    return (ElevatorPitchMotor.getSensorCollection().isRevLimitSwitchClosed() == 1);
  }

  public ControlMode getPitchState() {
    return ElevatorPitchMotor.getControlMode();
  }

  /**
   * 
   * @return
   */
  public ControlMode getElevatorState() {
    return ElevatorMotor.getControlMode();
  }

  // Control Cycles

  /**
   * Control Elevator Pitch:
   * This is the interface to control the motor to control the pitch of the elevator.
   * Ran in periodic, use set interface to control.
   */
  private void controlElevatorPitch() {
    ElevatorPitchMotor.set(setPitchControlMethod, setPitchInput);

  }

  private void controlElevator() {
    ElevatorMotor.set(setElevatorControlMethod, setElevatorInput);
    if (setElevatorControlMethod == ControlMode.MotionMagic) {
      setPosition = false;
    }
  }

  /**
   * This method will be called once per robot periodic/autonmous session at
   * initiation
   */
  public void init_periodic() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("setPosition", setPosition);
    controlElevatorPitch();
    controlElevator();
    SmartDashboard.putNumber("Elevator Height: ", ElevatorMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Elevator Top Pos", topPos);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
