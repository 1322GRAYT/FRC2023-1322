package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.configArmLift;
import frc.robot.calibrations.configArmPitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Finish documentation

public class LiftElevator extends SubsystemBase {

  public boolean setPosition = false;

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
    final var ConfigArmLift = new configArmLift();
    final var ConfigArmPitch = new configArmPitch();
    ElevatorMotor.configAllSettings(ConfigArmLift._fx);
    ElevatorMotor.setInverted(true);
    ElevatorMotor.setNeutralMode(NeutralMode.Brake);

    ElevatorPitchMotor.configAllSettings(ConfigArmPitch._fx);
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
    if (!setPosition) {
      setElevatorControlMethod = mode;
      setElevatorInput = setValue;
    }
  }

  public void setElevatorPercentPower(double setPower) {
    if (!setPosition) {
      setElevatorControlMethod = ControlMode.PercentOutput;
      setElevatorInput = -setPower;
    }
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
   * This is the interface to control solenoids to control the pitch of the
   * elevator.
   * Ran in periodic, use set interface to control.
   */
  private void controlElevatorPitch() {
    ElevatorPitchMotor.set(setPitchControlMethod, setPitchInput);

  }

  private void controlElevator() {
    ElevatorMotor.set(setElevatorControlMethod, setElevatorInput);
    while (setPosition &&Math.abs(ElevatorMotor.getSelectedSensorPosition(0)-setElevatorInput)> 2000) {
      // do nothing!!!
      SmartDashboard.putBoolean("Doing move!", setPosition);
    }
    SmartDashboard.putBoolean("Doing move!", false);
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
