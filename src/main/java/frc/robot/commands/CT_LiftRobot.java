// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RFSLIB;
import frc.robot.subsystems.LiftSubsystem.controlState;
import frc.robot.calibrations.K_LIFT;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CT_LiftRobot extends CommandBase {
  private LiftSubsystem liftSubsystem;
  private XboxController auxStick;
  private Timer delayTmr;

  private double liftPwr;
  private int dPadPos;

  /** Creates a new CT_LiftRobot. */
  public CT_LiftRobot(LiftSubsystem liftSubsystem, XboxController auxStick) {
    this.liftSubsystem = liftSubsystem;
    this.auxStick = auxStick;
    liftPwr = 0;
    delayTmr = new Timer();
    addRequirements(liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot Lift is Armed!");
    liftSubsystem.setLiftControlState(controlState.Init);
    delayTmr.stop();
    delayTmr.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dPadPos = auxStick.getPOV();

    liftPwr = -auxStick.getLeftY();
    liftPwr = RFSLIB.ApplyDB_Scld(liftPwr, K_LIFT.KeLIFT_r_CntlrDeadBandThrsh, 1.0);

    
    switch (liftSubsystem.getLiftControlState()) {
      case Init: {
        if ((dPadPos > 350 || dPadPos < 10) && dPadPos != -1){ // D-Pad Up
          liftSubsystem.setLiftControlState(controlState.ExtendFwdArmUp);
          liftSubsystem.getLiftTrackSlnd().set(true);
          liftSubsystem.getCameraSlnd().set(true);
        }
        break;
      }

      case ExtendFwdArmUp: {
        if (liftSubsystem.detectTrackLimitFront() == true){ 
          liftSubsystem.setLiftControlState(controlState.RetractMid);
        }
        break;
      }

      case RetractMid: {
        if (liftSubsystem.detectTrackMidTrigger() == true){ 
          liftSubsystem.setLiftControlState(controlState.RetractRear);
          liftSubsystem.getLiftTrackSlnd().set(false);
        }
        break;
       }

      case RetractRear: {
        if (liftSubsystem.detectTrackLimitRear() == true){ 
          liftSubsystem.setLiftControlState(controlState.ExtendFwdArmDwn);
          delayTmr.start();
        }
        break;
      }

      case ExtendFwdArmDwn: {
        if (delayTmr.get() >= K_LIFT.KeLIFT_t_RaiseTrackRearSwDlyTme){ 
          liftSubsystem.setLiftControlState(controlState.ExtendFwdArmUp);
          liftSubsystem.getLiftTrackSlnd().set(true);
          delayTmr.stop();
          delayTmr.reset();
        }
        break;
      }

      default: break;

    }

    liftSubsystem.runLiftAtPwr(liftPwr);

    SmartDashboard.putBoolean("LiftSwFront: ",  liftSubsystem.detectTrackLimitFront());
    SmartDashboard.putBoolean("LiftSwMid: ",    liftSubsystem.detectTrackMidTrigger());
    SmartDashboard.putBoolean("LiftSwBack: ",   liftSubsystem.detectTrackLimitRear());
    SmartDashboard.putString("LiftCtrlSt: ",    liftSubsystem.getLiftControlState().toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
