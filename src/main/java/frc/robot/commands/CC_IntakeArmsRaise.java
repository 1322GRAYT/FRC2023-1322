// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.calibrations.K_INTK;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CC_IntakeArmsRaise extends CommandBase {
  private IntakeSubsystem intakeSubsystem; 
  private Timer exitTmr;

  /** Creates a new CC_IntakeArmsRaise. */
  public CC_IntakeArmsRaise(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    exitTmr = new Timer();
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    intakeSubsystem.raiseIntakeArms();
    exitTmr.reset();
    exitTmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    exitTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (exitTmr.get() > K_INTK.KeINTK_t_IntakeArmRaiseLowerDlyCmd);
  }
}
