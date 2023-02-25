package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_LIFT;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorSubsystem extends SubsystemBase {
//this is to do the floor grab, rotate and drop....


/*
there are 4 floor sensors, 1-4
2 air floor grabs
floor motor
*/

  private WPI_TalonFX FloorMotor = new WPI_TalonFX(Constants.LIFT_MOTOR, "rio");
  
  //private Solenoid floorGrab0 = new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_LIFT_CONE_GRAB);

  private Solenoid FloorGrabSolenoid0 = new Solenoid(PneumaticsModuleType.REVPH,Constants.PNEUMATIC_FLOOR_GRAB_0);
  private Solenoid FloorGrabSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH,Constants.PNEUMATIC_FLOOR_GRAB_1);


   public enum GrabState {
    Open,
    Closed
   };


  GrabState state;

  public GrabState getState() { 
    return state;
  }
  
  public void grabToggle() {
    if (state == GrabState.Open) grabClose();
    if (state == GrabState.Closed) grabOpen();

  }

  public void grabOpen() {
    FloorGrabSolenoid0.set(false);
    FloorGrabSolenoid1.set(false);
  }

  public void grabClose() {
    FloorGrabSolenoid0.set(true);
    FloorGrabSolenoid1.set(true);  }


    public void liftDropReset() {
      //move the lift to the top
      while (! detectTrackLimitRear() ) {
        runLiftAtPwr(1);
      }
      haltLift();

      // drop the cone
      grabOpen();
      try {
        Thread.sleep(200);
      }
      catch (InterruptedException ie) {
      }

      //move the lift to the bottom
      while (! detectTrackLimitFront() ) {
        runLiftAtPwr(-1);
      }
      haltLift();
    }

  private DigitalInput TrackMidTrig = new DigitalInput(Constants.SW_LIFT_TRACK_TRIG);

  /********************************/
  /* LiftSubsystem Constructor */
  /********************************/
  public FloorSubsystem() {

    state = GrabState.Open;
    /*****************************************************************/
    /* Lift Motor Controller Configurations */
    /*****************************************************************/
    FloorMotor.configFactoryDefault();
    FloorMotor.setSensorPhase(false);
    FloorMotor.setInverted(false);
    FloorMotor.setNeutralMode(NeutralMode.Brake);

    FloorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, (int) 0, (int) 0);
    FloorMotor.setSelectedSensorPosition(0);
    FloorMotor.config_kP(0, K_LIFT.KeLIFT_K_Prop);
    FloorMotor.config_kI(0, K_LIFT.KeLIFT_K_Intgl);
    FloorMotor.config_kD(0, K_LIFT.KeLIFT_K_Deriv);
    FloorMotor.config_IntegralZone(0, K_LIFT.KeLIFT_r_IntglErrMaxEnbl);
    FloorMotor.config_kF(0, K_LIFT.KeLIFT_K_FdFwd);

    FloorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    FloorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

  }

  /**
   * Method: getLiftMtr - Robot Lift System - Gets the Robot Lift Motor Object
   * 
   * @return ShooterMotor; (WPI_TalonFX: Lift Motor Object)
   */
  public WPI_TalonFX getLiftMtr() {
    return FloorMotor;
  }

  public double getLiftPstn() {
    return getLiftMtr().getSelectedSensorPosition();
  }

  public void runLiftAtSpd(double speed) {
    getLiftMtr().set(ControlMode.Velocity, speed);
  }

  public void runLiftAtPwr(double pwr) {
    getLiftMtr().set(ControlMode.PercentOutput, pwr);
  }

  public void haltLift() {
    getLiftMtr().set(ControlMode.PercentOutput, 0);
  }

  public void pidLiftPstn(boolean activate, double pstn) {
    if (activate == true) {
      getLiftMtr().set(ControlMode.Position, pstn);
    } else {
      haltLift();
    }
  }

  public boolean detectTrackLimitFront() {
    boolean limitDetected = false;
    if (FloorMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1) {
      limitDetected = true;
    }
    return limitDetected;
  }

  public boolean detectTrackLimitRear() {
    boolean limitDetected = false;
    if (FloorMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      limitDetected = true;
    }
    return limitDetected;
  }


  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
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
