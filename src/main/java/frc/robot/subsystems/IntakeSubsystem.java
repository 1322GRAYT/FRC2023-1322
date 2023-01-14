package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_INTK;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public enum slctArm {
    Left,
    Right,
    Front,
    Rear
  }

  public enum controlState {
    Init,
    SeekBall1,
    GrabBall1,
    HoldBall1,
    SeekBall2,
    GrabBall2,
    HoldBall2
  }



  private WPI_TalonFX AdvanceMotor = new WPI_TalonFX(Constants.BALL_MTR_ADVANCE, "rio");

  private TalonSRX IntakeMotor = new TalonSRX(Constants.BALL_MTR_INTAKE);

  private PneumaticHub hub = new PneumaticHub(1);

  private Solenoid[] BallIntakeArm = new Solenoid[] {
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_LT),
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_RR),
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_RT),
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_FT)
  };


/*
  private DigitalInput[] BallArmDetect = new DigitalInput[] {
    new DigitalInput(Constants.SW_BALL_INTAKE_LT),
    new DigitalInput(Constants.SW_BALL_INTAKE_RR),
    new DigitalInput(Constants.SW_BALL_INTAKE_RT),
    new DigitalInput(Constants.SW_BALL_INTAKE_FT)
  };
*/  

  private DigitalInput BallArmDetectLT = new DigitalInput(Constants.SW_BALL_INTAKE_LT);
  private DigitalInput BallArmDetectRT = new DigitalInput(Constants.SW_BALL_INTAKE_RT);
  private DigitalInput BallArmDetectFT = new DigitalInput(Constants.SW_BALL_INTAKE_FT);
  private DigitalInput BallArmDetectRR = new DigitalInput(Constants.SW_BALL_INTAKE_RR);

  private DigitalInput BallAdvance1 = new DigitalInput(Constants.SW_BALL_ADVANCE_1);
  private DigitalInput BallAdvance2 = new DigitalInput(Constants.SW_BALL_ADVANCE_2);


  private Timer detectArmTmr = new Timer();  
  private Timer detectAdv1Tmr = new Timer();
  private Timer detectAdv2Tmr = new Timer();


  private boolean switchStateArmLeft;
  private boolean switchStateArmRight;
  private boolean switchStateArmFront;
  private boolean switchStateArmRear;
  private boolean switchStateArmComb;
  private boolean switchStateArmFilt;  
  private boolean switchStateAdvPstn1;
  private boolean switchStateAdvPstn1Filt;
  private boolean switchStateAdvPstn2;
  private boolean switchStateAdvPstn2Filt;

  private boolean ballCaptureInProgress;
  private boolean ballCapturedPstn1;
  private boolean ballCapturedPstn2;

  private controlState ballIntakeCtrlSt;

  private int instrUpdCnt;



  /********************************/
  /* BallSubsystem Constructor    */
  /********************************/
  public IntakeSubsystem() {

    switchStateArmLeft      = false;
    switchStateArmRight     = false;
    switchStateArmFront     = false;
    switchStateArmRear      = false;
    switchStateArmComb      = false;
    switchStateArmFilt      = false;
    switchStateAdvPstn1     = false;
    switchStateAdvPstn1Filt = false;
    switchStateAdvPstn2     = false;
    switchStateAdvPstn2Filt = false;


    detectArmTmr.reset();
    detectAdv1Tmr.reset();
    detectAdv2Tmr.reset();

    instrUpdCnt = (int)0;

    hub.clearStickyFaults();


 /*****************************************************************/
 /* Ball Intake Motor Controller Configurations                   */
 /*****************************************************************/    
    IntakeMotor.configFactoryDefault();
    IntakeMotor.setSensorPhase(true);
    IntakeMotor.setInverted(true);	
    IntakeMotor.setNeutralMode(NeutralMode.Coast);

    IntakeMotor.config_kP(0, K_INTK.KeINTK_K_InProp);
    IntakeMotor.config_kI(0, K_INTK.KeINTK_K_InIntgl);
    IntakeMotor.config_kD(0, K_INTK.KeINTK_K_InDeriv);
    IntakeMotor.config_IntegralZone(0, K_INTK.KeINTK_r_InIntglErrMaxEnbl);
    IntakeMotor.config_kF(0, K_INTK.KeINTK_K_InFdFwd);


  /*****************************************************************/
  /* Ball Advance Motor Controller Configurations                  */
  /*****************************************************************/  
    AdvanceMotor.configFactoryDefault();
    AdvanceMotor.setSensorPhase(false);
    AdvanceMotor.setInverted(false);	
    AdvanceMotor.setNeutralMode(NeutralMode.Brake);

    AdvanceMotor.config_kP(0, K_INTK.KeINTK_K_AdvProp);
    AdvanceMotor.config_kI(0, K_INTK.KeINTK_K_AdvIntgl);
    AdvanceMotor.config_kD(0, K_INTK.KeINTK_K_AdvDeriv);
    AdvanceMotor.config_IntegralZone(0, K_INTK.KeINTK_r_AdvIntglErrMaxEnbl);
    AdvanceMotor.config_kF(0, K_INTK.KeINTK_K_AdvFdFwd);

    ballCaptureInProgress = false;
    setBallCapturedPstn1(false);
    setBallCapturedPstn2(false);
    ballIntakeCtrlSt = controlState.Init;

  }



  /**
   * Method: getIntakeMtr - Shooter Drive System - Gets the Master Ball Feed Intake Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Ball Feed Intake Motor Object)
   */  
  public TalonSRX getIntakeMtr() {
    return IntakeMotor;
  }

  public void stopIntakeMtr(){
    getIntakeMtr().set(ControlMode.Disabled, 0);
  }

  public double getIntakeSpd(){
    return getIntakeMtr().getSelectedSensorVelocity();
  }

  public void runIntakeAtPwr(double power) {
    getIntakeMtr().set(ControlMode.PercentOutput,power);
  }

  public void runIntakeAtSpd(double speed) {
    getIntakeMtr().set(ControlMode.Velocity,speed);
  }

  public void pidIntakeSpd(boolean activate){
    if (activate == true) {
      getIntakeMtr().set(ControlMode.Velocity,K_INTK.KeINTK_n_TgtIntakeCmdFeed);
    } else {
      getIntakeMtr().set(ControlMode.Velocity,0);
    }
  }




  /**
   * Method: getAdvanceMtr - Shooter Drive System - Gets the Master Ball Feed Advance Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Ball Feed Advance Motor Object)
   */  
  public WPI_TalonFX getAdvanceMtr() {
    return AdvanceMotor;
  }

  public void stopAdvanceMtr(){
    getAdvanceMtr().set(TalonFXControlMode.Disabled, 0);
  }

  public double getAdvanceSpd(){
    return getAdvanceMtr().getSelectedSensorVelocity();
  }

  public void runAdvanceAtPwr(double power) {
    getAdvanceMtr().set(TalonFXControlMode.PercentOutput,power);
  }

  public void runAdvanceAtSpd(double speed) {
    getAdvanceMtr().set(TalonFXControlMode.Velocity,speed);
    if(speed == 0) getAdvanceMtr().disable();
  }

  public void pidAdvanceSpd(boolean activate){
    if (activate == true) {
      getAdvanceMtr().set(TalonFXControlMode.Velocity,K_INTK.KeINTK_n_TgtAdvanceCmdShoot);
    } else {
      getAdvanceMtr().set(TalonFXControlMode.Velocity,0);
    }
  }





  public void lowerIntakeArms() {
    int i;
    for (i = 0; i < 4; i++) {
      BallIntakeArm[i].set(false);
    }
  }


  public void raiseIntakeArms() {
    int i;
    for (i = 0; i < 4; i++) {
      BallIntakeArm[i].set(true);
    }
  }


/*
  public boolean detectBallAtArm() {
    boolean ballDetected = false;
    int i;
    for (i = 0; i < 4; i++) {
      if (BallArmDetect[i].get() == false) {
        ballDetected = true;
      }
    }
    return ballDetected;
  }
*/

  public boolean detectBallArmLeft() {
    return (BallArmDetectLT.get());
  }

  private boolean detectBallArmRight() {
    return (BallArmDetectRT.get());
  }

  private boolean detectBallArmFront() {
    return (BallArmDetectFT.get());
  }

  private boolean detectBallArmRear() {
    return (BallArmDetectRR.get());
  }

  private boolean detectBallAdvance1() {
    return (!BallAdvance1.get());
  }

  private boolean detectBallAdvance2() {
    return (!BallAdvance2.get());
  }




  public boolean getBallArmArrayFilt() {
    return (switchStateArmFilt);
  }


  public boolean getBallAdvPstn1() {
    return (switchStateAdvPstn1);
  }

  public boolean getBallAdvPstn2() {
    return (switchStateAdvPstn2);
  }

  public boolean getBallAdvPstn1Filt() {
    return (switchStateAdvPstn1Filt);
  }

  public boolean getBallAdvPstn2Filt() {
    return (switchStateAdvPstn2Filt);
  }

/*
  switchStateArmLeft  = false;
  switchStateArmRight = false;
  switchStateArmFront = false;
  switchStateArmRear  = false;
*/


  public void setBallCaptureInProgress(boolean captureInProgress) {
    ballCaptureInProgress = captureInProgress;
  }

  public boolean getBallCaptureInProgress() {
    return (ballCaptureInProgress);
  }



  public void setBallCapturedPstn1(boolean ballCaptured) {
    ballCapturedPstn1 = ballCaptured;
  }

  public boolean getBallCapturedPstn1() {
    return (ballCapturedPstn1);
  }



  public void setBallCapturedPstn2(boolean ballCaptured) {
    ballCapturedPstn2 = ballCaptured;
  }

  public boolean getBallCapturedPstn2() {
    return (ballCapturedPstn2);
  }




  public void setBallIntakeCtrlSt(controlState  intakeCtrlSt) {
    ballIntakeCtrlSt = intakeCtrlSt;
  }

  public controlState getBallIntakeCtrlSt() {
    return (ballIntakeCtrlSt);
  }

  



  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at initiation
    setBallIntakeCtrlSt(controlState.Init);
  }

  @Override
  public void periodic() {
   // System.out.println("Start IntakeSubsystem.");

    switchStateArmLeft  = detectBallArmLeft();
    switchStateArmRight = detectBallArmRight();
    switchStateArmFront = detectBallArmFront();
    switchStateArmRear  = detectBallArmRear();
    
    if (switchStateArmLeft || switchStateArmRight ||
        switchStateArmFront ||switchStateArmRear) {
      switchStateArmComb = true;      
    } else {
      switchStateArmComb = false;
    }

    if (switchStateArmComb == true) {
      detectArmTmr.start();
    }  else {
      detectArmTmr.stop();
      detectArmTmr.reset();
    }

    if (switchStateArmComb != switchStateArmFilt) {
      if (switchStateArmComb == true) {
        if (detectArmTmr.get() >= K_INTK.KeINTK_t_IntakeArmDtctTme) {
          switchStateArmFilt = true;
        }
      } else {
        switchStateArmFilt = false;
      }      
    }


    switchStateAdvPstn1 = detectBallAdvance1();

    if (switchStateAdvPstn1) {
      detectAdv1Tmr.start();
    }  else {
      detectAdv1Tmr.stop();
      detectAdv1Tmr.reset();
      setBallCapturedPstn1(false);
    }  

    if (switchStateAdvPstn1 != switchStateAdvPstn1Filt) {
      if (switchStateAdvPstn1 == true) {
        if (detectAdv1Tmr.get() >= K_INTK.KeINTK_t_IntakeAdv1DtctTme) {
          switchStateAdvPstn1Filt = true;
        }
      } else {
        switchStateAdvPstn1Filt = false;
      }      
    }

    if (detectAdv1Tmr.get() >= K_INTK.KeINTK_t_IntakeAdv2CaptTme) {
      setBallCapturedPstn1(true);
    }


    switchStateAdvPstn2 = detectBallAdvance2();

    if (switchStateAdvPstn2) {
      detectAdv2Tmr.start();
    }  else {
      detectAdv2Tmr.stop();
      detectAdv2Tmr.reset();
      setBallCapturedPstn2(false);
    }

    if (switchStateAdvPstn2 != switchStateAdvPstn2Filt) {
      if (switchStateAdvPstn2 == true) {
        if (detectAdv2Tmr.get() >= K_INTK.KeINTK_t_IntakeAdv2DtctTme) {
          switchStateAdvPstn2Filt = true;
        }
      } else {
        switchStateAdvPstn2Filt = false;
      }      
    }

    if (detectAdv2Tmr.get() >= K_INTK.KeINTK_t_IntakeAdv2CaptTme) {
      setBallCapturedPstn2(true);
    }



    if (instrUpdCnt == (int)0) {
      SmartDashboard.putBoolean("IntakeCaptInProg: ", ballCaptureInProgress);
      SmartDashboard.putBoolean("IntakeBallCapt1: ",  getBallCapturedPstn1());
      SmartDashboard.putBoolean("IntakeBallCapt2: ",  getBallCapturedPstn2());
      SmartDashboard.putBoolean("IntakeArmLTDtctd: ", switchStateArmLeft);
      SmartDashboard.putBoolean("IntakeArmRTDtctd: ", switchStateArmRight);
      SmartDashboard.putBoolean("IntakeArmFTDtctd: ", switchStateArmFront);
      SmartDashboard.putBoolean("IntakeArmRRDtctd: ", switchStateArmRear);
      SmartDashboard.putBoolean("IntakeArmDtctdC: ",  switchStateArmComb);
      SmartDashboard.putBoolean("IntakeArmDtctdF: ",  switchStateArmFilt);
      SmartDashboard.putBoolean("IntakeAdv1Dtctd: ",  switchStateAdvPstn1);
      SmartDashboard.putBoolean("IntakeAdv1DtctdF: ", switchStateAdvPstn1Filt);
      SmartDashboard.putBoolean("IntakeAdv2Dtctd: ",  switchStateAdvPstn2);
      SmartDashboard.putBoolean("IntakeAdv2DtctdF: ", switchStateAdvPstn2Filt);

      SmartDashboard.putNumber("IntakeArmTmr: ",      detectArmTmr.get());
      SmartDashboard.putNumber("IntakeAdv1Tmr: ",     detectAdv1Tmr.get());
      SmartDashboard.putNumber("IntakeAdv2Tmr: ",     detectAdv2Tmr.get());

      SmartDashboard.putString("IntakeCtrlSt: ",      ballIntakeCtrlSt.toString());
    }
    instrUpdCnt++;
    if (instrUpdCnt >= (int)10) {
      instrUpdCnt = (int) 0;  
    }


    

  //  System.out.println("End IntakeSubsystem.");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }










}
