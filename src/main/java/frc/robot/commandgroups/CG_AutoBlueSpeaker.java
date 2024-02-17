// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.CC_IntakeOff;
import frc.robot.commands.CC_IntakeOn;
import frc.robot.commands.CC_LiftLoadPos;
import frc.robot.commands.CC_TiltLoadPos;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_AutoBlueSpeaker extends SequentialCommandGroup {
  public CG_AutoBlueSpeaker(TiltSubsystem tiltSubsystem, LiftSubsystem liftSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      (new CC_TiltLoadPos(tiltSubsystem)),
      (new CC_LiftLoadPos(liftSubsystem)),
      (new CC_IntakeOn(intakeSubsystem)),
      (new CC_IntakeOff(intakeSubsystem))
    );
    System.out.println("CGA_BlueSpeaker Invoked.");
  }
}
