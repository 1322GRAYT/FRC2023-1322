// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.calibrations.K_INTK;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.controlState;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CA_IntakeBall extends CommandBase {


  private IntakeSubsystem intakeSubsystem;
  private Timer seekTmr;
  private Timer endTmr;
  private int instrUpdCnt;



  /** Creates a new CT_IntakeBalls. */
  public CA_IntakeBall(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    seekTmr = new Timer();
    endTmr = new Timer();
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot, Auto Grab Ball!");
    intakeSubsystem.runIntakeAtPwr(K_INTK.KeINTK_r_TgtIntakePwrFeed);
    intakeSubsystem.getAdvanceMtr().setNeutralMode(NeutralMode.Brake);
    seekTmr.reset();
    seekTmr.stop();
    endTmr.reset();
    endTmr.stop();
    intakeSubsystem.setBallIntakeCtrlSt(controlState.Init);
    instrUpdCnt = (int)0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*****************************/  
    /* Ball Intake State Machine */
    /*****************************/
    switch (intakeSubsystem.getBallIntakeCtrlSt()) {
      case Init: {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.SeekBall1);
          seekTmr.start();
  
        break;
      }
  
      case SeekBall1: {
        if (intakeSubsystem.getBallArmArrayFilt() == true) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.GrabBall1);
          intakeSubsystem.lowerIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(true);
          seekTmr.stop();
        }
  
        break;
      }
  
      case GrabBall1: {
        if (intakeSubsystem.getBallAdvPstn1() == true) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall1);
          intakeSubsystem.runAdvanceAtPwr(K_INTK.KeINTK_r_TgtAdvancePwrFeed);
          intakeSubsystem.setBallCaptureInProgress(false);
          endTmr.start();          
        }  
  
        break;
      }
      
      case HoldBall1: {
        if (intakeSubsystem.getBallAdvPstn2Filt() == true) {
          intakeSubsystem.stopAdvanceMtr();
        /* Stop Advance Motor - Hold Balls with Arms Down until Shooter Fires */
        }

        break;
      }
        
      default: break;
    }
 
    if (instrUpdCnt == (int)0) {
      SmartDashboard.putNumber("IntakeArmSpd: ",      intakeSubsystem.getIntakeSpd());
      SmartDashboard.putNumber("IntakeAdvSpd: ",      intakeSubsystem.getAdvanceSpd());
    }
    instrUpdCnt++;
    if (instrUpdCnt >= (int)10)
      instrUpdCnt = (int) 0;

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntakeMtr();
    intakeSubsystem.stopAdvanceMtr();
    seekTmr.stop();
    endTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMet1 = false;
    boolean condMet2 = (seekTmr.get() >= K_INTK.KeINTK_t_AutoBallSeekTmeOut);
    if ((intakeSubsystem.getBallIntakeCtrlSt() == controlState.HoldBall1) ||
        (endTmr.get() >= K_INTK.KeINTK_t_AutoBallHoldEndTmeOut)) {
          condMet1 = true;  
        }
    return (condMet1 || condMet2);
  }
}
