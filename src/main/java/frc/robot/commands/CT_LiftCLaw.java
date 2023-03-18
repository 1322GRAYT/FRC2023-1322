// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftClaw;
import frc.robot.subsystems.LiftClaw.ClawState;

public class CT_LiftCLaw extends CommandBase {
  private LiftClaw liftClaw;
  private XboxController auxStick;
  private XboxController driverStick;

  /** Creates a new CT_LiftCLaw. */
  public CT_LiftCLaw(LiftClaw liftClaw, XboxController auxStick, XboxController driverStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.liftClaw = liftClaw;
    this.auxStick = auxStick;
    this.driverStick = driverStick;

    addRequirements(liftClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!liftClaw.getGrabSensor() && auxStick.getYButtonPressed()){
      liftClaw.setIntakeMotorPower(1.0);
    }
    else if (!liftClaw.getGrabSensor() && auxStick.getAButtonPressed()){
      liftClaw.setIntakeMotorPower(-1.0);
      }
    if (liftClaw.getGrabSensor() || auxStick.getAButtonReleased() || auxStick.getYButtonReleased()) {
      liftClaw.setIntakeMotorPower(0);
    }

    if(liftClaw.getClawState() == ClawState.Closed && auxStick.getBButtonPressed()){
      liftClaw.setClawState(ClawState.Open);
    }
    else if (liftClaw.getClawState() == ClawState.Open && (auxStick.getBButtonPressed() || (liftClaw.getGrabSensor() && auxStick.getBButtonReleased()))){
      liftClaw.setClawState(ClawState.Closed);
    }
 

    //liftClaw.setClawState(driverStick.getRightBumper() ? ClawState.Open : ClawState.Closed);
    liftClaw.setPitchSetPoint(auxStick.getRightY() * 0.25); // TODO: transfer scaling to motor controller maybe
    liftClaw.setYawPower(auxStick.getRightX());
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
