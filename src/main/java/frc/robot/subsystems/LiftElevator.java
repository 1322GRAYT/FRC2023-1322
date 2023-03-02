package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.configArmLift;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


// TODO: Finish documentation

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

  private pitchState elevatorPitchState = pitchState.Back;
  private double setInput = 0;
  private ControlMode setControlMethod = ControlMode.PercentOutput;

  // Resources
  private WPI_TalonFX ElevatorMotor = new WPI_TalonFX(Constants.ELEVATOR_MOTOR);
  private DoubleSolenoid InitialArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_ARM_TILT_INITIAL_0, Constants.PNEUMATIC_ARM_TILT_INITIAL_1);
  private DoubleSolenoid SecondaryArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PNEUMATIC_ARM_TILT_SECONDARY_0, Constants.PNEUMATIC_ARM_TILT_SECONDARY_1);

  private DigitalInput TrackMidTrig = new DigitalInput(Constants.SW_LIFT_TRACK_TRIG);

  /**
   * Contructor
   */
  public LiftElevator() {
    elevatorPitchState = pitchState.Back;
    final var ConfigArmLift = new configArmLift();
    ElevatorMotor.configAllSettings(ConfigArmLift._fx);
    ElevatorMotor.setInverted(true);
  }

  // Set Interfaces

  public void haltElevator() {
    ElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Interface to set elevator pitch
   * 
   * @param armControlState Enumerator to set the three positions of the elevator
   *                        pitch
   */
  public void setElevatorPitch(pitchState armControlState) {
    this.elevatorPitchState = armControlState;
  }

  /**
   * Sets control mode to Motion Magic
   * TODO: develop tick conversion factor, approx 5500 ticks per inch
   * @param setPoint Set point for motion magic, currently set as ticks. 5500 ticks per inch approx
   */
  public void setElevatorPosition(double setPoint) {
    setControlMethod = ControlMode.MotionMagic;
    setInput = setPoint;
  }


  public void setElevatorControl(ControlMode mode, double setValue) {
    setControlMethod = mode;
    setInput = setValue;
  }

  public void setElevatorPercentPower(double setPower) {
    setControlMethod = ControlMode.PercentOutput;
    setInput = -setPower;
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
   * Gets elevator pitch enumerator
   * 
   * @return Position of the elevator pitch, only the setting. May not be current
   *         exact position
   */
  public pitchState getElevatorPitch() {
    return elevatorPitchState;
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

  /**
   * 
   * @return
   */
  public pitchState getControlState() {
    return elevatorPitchState;
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
   * This is the interface to control solenoids to control the pitch of the
   * elevator.
   * Ran in periodic, use set interface to control.
   */
  private void controlElevatorPitch() {

    Value setInitial = kOff;
    Value setSecondary = kOff;

    switch (elevatorPitchState) {
      case Back:
        setInitial = kForward;
        setSecondary = kForward;
        break;
      case Mid:
        setInitial = kForward;
        setSecondary = kReverse;
        break;
      case Front:
        setInitial = kReverse;
        setSecondary = kReverse;
        break;
    }
    InitialArmSolenoid.set(setInitial);
    SecondaryArmSolenoid.set(setSecondary);
  }

  private void controlElevator() {
    ElevatorMotor.set(setControlMethod, setInput);
  }

  /**
   * This method will be called once per robot periodic/autonmous session at
   * initiation
   */
  public void init_periodic() {
    setElevatorPitch(pitchState.Back);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    controlElevatorPitch();
    controlElevator();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
