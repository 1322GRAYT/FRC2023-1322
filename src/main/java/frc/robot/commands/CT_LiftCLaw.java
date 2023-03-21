// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftClaw;
import frc.robot.subsystems.LiftClaw.ClawState;

public class CT_LiftCLaw extends CommandBase {
  private LiftClaw liftClaw;

  Supplier<Double> pitchPowerControl, yawPowerControl;
  Supplier<Boolean> clawOpenControl;

  /** Creates a new CT_LiftCLaw. */
  public CT_LiftCLaw(LiftClaw liftClaw, Supplier<Double> pitchPowerControl, Supplier<Double> yawPowerControl, Supplier<Boolean> clawOpenControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.liftClaw = liftClaw;
    this.pitchPowerControl = pitchPowerControl;
    this.yawPowerControl = yawPowerControl;
    this.clawOpenControl = clawOpenControl;

    addRequirements(liftClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  boolean stateReleased = true;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var clawButtonState = clawOpenControl.get();

    if(liftClaw.getClawState() == ClawState.Closed && clawButtonState){
      liftClaw.setClawState(ClawState.Open);
    }
    else if (liftClaw.getClawState() == ClawState.Open && (clawButtonState || (liftClaw.getGrabSensor() && stateReleased))){
      liftClaw.setClawState(ClawState.Closed);
    }

    stateReleased = !clawButtonState;
 
    if(Math.abs(pitchPowerControl.get()) - 0.1 > 0 || liftClaw.getPiControlMode() == ControlMode.PercentOutput){
      liftClaw.setPitchPower(pitchPowerControl.get()); // TODO: transfer scaling to motor controller maybe
    }

    liftClaw.setYawPower(yawPowerControl.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
