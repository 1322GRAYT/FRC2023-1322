package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_LIFT;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

public enum controlState {
  Init,
  ExtendFwdArmDwn,
  ExtendFwdArmUp,
  RetractMid,
  RetractRear
}

private WPI_TalonFX LiftMotor = new WPI_TalonFX(Constants.LIFT_MTR, "rio");

private Solenoid LiftTrackSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_LIFT_TRACK);

private Solenoid CameraSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_SHOOTER_CAMERA);

private DigitalInput TrackMidTrig = new DigitalInput(Constants.SW_LIFT_TRACK_TRIG);

private controlState liftCntrlSt;



  /********************************/
  /* LiftSubsystem Constructor    */
  /********************************/
  public LiftSubsystem() {

  
 /*****************************************************************/
 /* Lift Motor Controller Configurations                          */
 /*****************************************************************/  
   LiftMotor.configFactoryDefault();
   LiftMotor.setSensorPhase(false);
   LiftMotor.setInverted(false);	
   LiftMotor.setNeutralMode(NeutralMode.Brake);

   LiftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,(int)0,(int)0);
   LiftMotor.setSelectedSensorPosition(0);   
   LiftMotor.config_kP(0, K_LIFT.KeLIFT_K_Prop);
   LiftMotor.config_kI(0, K_LIFT.KeLIFT_K_Intgl);
   LiftMotor.config_kD(0, K_LIFT.KeLIFT_K_Deriv);
   LiftMotor.config_IntegralZone(0, K_LIFT.KeLIFT_r_IntglErrMaxEnbl);
   LiftMotor.config_kF(0, K_LIFT.KeLIFT_K_FdFwd);

   LiftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
   LiftMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

   liftCntrlSt = controlState.Init;

  }


  /**
   * Method: getLiftMtr - Robot Lift System - Gets the Robot Lift Motor Object 
   * @return ShooterMotor; (WPI_TalonFX: Lift Motor Object)
   */  
  public WPI_TalonFX getLiftMtr() {
    return LiftMotor;
  }

  /**
   * Method: getLiftTrackSlnd - Robot Lift System - Gets the Robot Lift Track Solenoid Object 
   * @return LiftTrackSolenoid; (Solenoid: Lift Track Solenoid Object)
   */  
  public Solenoid getLiftTrackSlnd() {
    return LiftTrackSolenoid;
  }

  /**
   * Method: getCameraSlnd - Robot Lift System - Gets the Robot Shooter Camera Solenoid Object 
   * @return CameraSolenoid; (WPI_TalonFX: Shooter Camera Solenoid Object)
   */  
  public Solenoid getCameraSlnd() {
    return CameraSolenoid;
  }



  public double getLiftPstn(){
    return getLiftMtr().getSelectedSensorPosition();
  }

  public void runLiftAtSpd(double speed) {
    getLiftMtr().set(ControlMode.Velocity,speed);
  }

  public void runLiftAtPwr(double pwr) {
    getLiftMtr().set(ControlMode.PercentOutput,pwr);
  }

  public void haltLift() {
    getLiftMtr().set(ControlMode.PercentOutput,0);
  }

  public void pidLiftPstn(boolean activate, double pstn){
    if (activate == true) {
      getLiftMtr().set(ControlMode.Position, pstn);
    } else {
      haltLift();
    }
  }


  public boolean detectTrackMidTrigger() {
    return (TrackMidTrig.get() == false);
  }  


  public boolean detectTrackLimitFront() {
    boolean limitDetected = false;
    if (LiftMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1) {
      limitDetected = true;    
    }
    return limitDetected;
  }   

  public boolean detectTrackLimitRear() {
    boolean limitDetected = false;
    if (LiftMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      limitDetected = true;    
    }
    return limitDetected;
  }   



  public void setLiftControlState(controlState controlSt) {
    liftCntrlSt = controlSt;
  }  

  public controlState getLiftControlState() {
    return liftCntrlSt;
  }  



  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at initiation  
    setLiftControlState(controlState.Init);
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
