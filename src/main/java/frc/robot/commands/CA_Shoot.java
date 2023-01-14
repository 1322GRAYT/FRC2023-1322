// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.*;
import frc.robot.calibrations.K_INTK;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CA_Shoot extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Camera cameraSubsystem;
  private slctGoal slctTgtGoal; // if true, Shooter is targeting high goal, if false, low goal.
  private Timer timeSinceBallsLeftAdvPstn;
  private double targetDistance;
  private double dsrdShooterSpeed;
  
  /** Creates a new CA_Shoot - Either High or Low Goal, selection passed as an argument. */
  public CA_Shoot(ShooterSubsystem shooterSubsystem,  IntakeSubsystem intakeSubsystem, Camera cameraSubsystem, slctGoal slctTgtGoal) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.cameraSubsystem = cameraSubsystem;
    this.slctTgtGoal = slctTgtGoal;
    timeSinceBallsLeftAdvPstn = new Timer();
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot, Shoot! Autonomous.");
    intakeSubsystem.getAdvanceMtr().setNeutralMode(NeutralMode.Coast);
    timeSinceBallsLeftAdvPstn.reset();
    timeSinceBallsLeftAdvPstn.stop();

    // Determine Desired Target Speed as a function of target distance
    targetDistance = cameraSubsystem.getDistanceToCenterOfHoop();
    if (slctTgtGoal == slctGoal.HiGoalCamera) {
      dsrdShooterSpeed = shooterSubsystem.dtrmnShooterSpdAuto(targetDistance);
    } else if (slctTgtGoal == slctGoal.HiGoalFar) {
      dsrdShooterSpeed = K_SHOT.KeSHOT_n_TgtLaunchCmdHiGoalCloseAuto;
    } else if (slctTgtGoal == slctGoal.HiGoalMid) {
      dsrdShooterSpeed = K_SHOT.KeSHOT_n_TgtLaunchCmdHiGoalMidAuto;
    } else if (slctTgtGoal == slctGoal.HiGoalClose) {
      dsrdShooterSpeed = K_SHOT.KeSHOT_n_TgtLaunchCmdHiGoalCloseAuto;
    } else {
      dsrdShooterSpeed = K_SHOT.KeSHOT_n_TgtLaunchCmdLoGoalAuto;
    }

    // Command Shooter to Desired Shooter Target Speed
    shooterSubsystem.runShooterAtSpd(dsrdShooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When shooter is at speed, feed the balls for shot, otherwise wait.
    shooterSubsystem.dtrmnShooterAtSpd(dsrdShooterSpeed);

    if (shooterSubsystem.isShooterAtSpd()) {
      System.out.println("Command Intakes!");
      intakeSubsystem.runAdvanceAtPwr(K_INTK.KeINTK_r_TgtAdvancePwrShoot);
      intakeSubsystem.runIntakeAtPwr(K_INTK.KeINTK_r_TgtIntakePwrShoot);
    } else {
      intakeSubsystem.stopAdvanceMtr();
      intakeSubsystem.stopIntakeMtr();
    }
    
    if ((intakeSubsystem.getBallAdvPstn1() == false) && (intakeSubsystem.getBallAdvPstn2() == false)) {
      timeSinceBallsLeftAdvPstn.start();      
    } else {
      timeSinceBallsLeftAdvPstn.reset();
      timeSinceBallsLeftAdvPstn.stop();        
    }
    if (timeSinceBallsLeftAdvPstn.get() >= K_SHOT.KeSHOT_t_IntakeArmRaiseDlyShoot) {
      intakeSubsystem.raiseIntakeArms();
    }

    if (K_SHOT.KeSHOT_b_DebugEnbl == true) {
      SmartDashboard.putNumber("Shooter Act Spd: ",    (shooterSubsystem.getSpd()));
      SmartDashboard.putNumber("Shooter Tgt Spd: ",    (dsrdShooterSpeed));
      SmartDashboard.putBoolean("Shooter At Spd: ",    (shooterSubsystem.isShooterAtSpd()));
      SmartDashboard.putNumber("Shooter Tgt Dist: ",   (targetDistance));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooter Cancel.");
     intakeSubsystem.stopAdvanceMtr();
     intakeSubsystem.stopIntakeMtr();
     shooterSubsystem.stopShooterMtr();
     timeSinceBallsLeftAdvPstn.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeSinceBallsLeftAdvPstn.get() >= K_SHOT.KeSHOT_t_PostLaunchRunTime);
  }
}
