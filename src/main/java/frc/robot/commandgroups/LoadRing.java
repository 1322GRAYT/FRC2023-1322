// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.LiftToLoadPosition;
import frc.robot.commands.TiltToLoadPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LoadRing extends SequentialCommandGroup {
  public LoadRing(TiltSubsystem tiltSubsystem, LiftSubsystem liftSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      (new TiltToLoadPosition(tiltSubsystem)),
      (new LiftToLoadPosition(liftSubsystem)),
      (new IntakeOn(intakeSubsystem)),
      (new IntakeOff(intakeSubsystem))
    );
    System.out.println("CG_LoadRing Invoked.");
  }
}
