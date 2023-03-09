// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LiftClaw;
import frc.robot.subsystems.LiftClaw.ClawState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CA_ToggleClaw extends InstantCommand {
  private LiftClaw liftClaw;

  public CA_ToggleClaw(LiftClaw liftClaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(liftClaw);
    this.liftClaw = liftClaw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    liftClaw.setClawState(liftClaw.getClawState() == ClawState.Open ? ClawState.Closed : ClawState.Open);
  }
}
