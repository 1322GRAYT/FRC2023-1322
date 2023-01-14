// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.calibrations.K_INTK;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CT_Shoot extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Camera cameraSubsystem;
  private boolean isHighGoal; // if true, Shooter is targeting high goal, if false, low goal.
  private Timer raiseArmTmr;
  private double targetDistance;
  private double dsrdShooterSpeed;
  
  /** Creates a new CT_Shoot - Either High or Low Goal, selection passed as an argument. */
  public CT_Shoot(ShooterSubsystem shooterSubsystem,  IntakeSubsystem intakeSubsystem, Camera cameraSubsystem, boolean isHighGoal) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.cameraSubsystem = cameraSubsystem;
    this.isHighGoal = isHighGoal;
    raiseArmTmr = new Timer();
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot, Shoot! Tele-Op.");
    intakeSubsystem.getAdvanceMtr().setNeutralMode(NeutralMode.Coast);
    raiseArmTmr.reset();
    raiseArmTmr.stop();

    // Determine Desired Target Speed as a function of target distance
    targetDistance = cameraSubsystem.getDistanceToCenterOfHoop();
    if (isHighGoal) {
      dsrdShooterSpeed = shooterSubsystem.dtrmnShooterSpdTeleOp(targetDistance);
    } else {
      dsrdShooterSpeed = K_SHOT.KeSHOT_n_TgtLaunchCmdLoGoal;
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
      intakeSubsystem.runAdvanceAtPwr(K_INTK.KeINTK_r_TgtAdvancePwrShoot);
      intakeSubsystem.runIntakeAtPwr(K_INTK.KeINTK_r_TgtIntakePwrShoot);
    } else {
      intakeSubsystem.stopAdvanceMtr();
      intakeSubsystem.stopIntakeMtr();
    }
    
    if ((intakeSubsystem.getBallAdvPstn1() == false) && (intakeSubsystem.getBallAdvPstn2() == false)) {
      raiseArmTmr.start();      
    } else {
      raiseArmTmr.reset();
      raiseArmTmr.stop();
    }
    if (raiseArmTmr.get() >= K_SHOT.KeSHOT_t_IntakeArmRaiseDlyShoot) {
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
     raiseArmTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
