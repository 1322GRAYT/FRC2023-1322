// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CA_Elevator;
import frc.robot.commands.CA_PitchElevator;
import frc.robot.commands.CA_ToggleClaw;
import frc.robot.subsystems.LiftClaw;
import frc.robot.subsystems.LiftElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_PlaceCone extends SequentialCommandGroup {
  /** Creates a new CG_PlaceCone. */
  public CG_PlaceCone(LiftClaw liftClaw, LiftElevator liftElevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CA_Elevator(liftElevator, true), new CA_PitchElevator(liftElevator, true), 
      new CA_ToggleClaw(liftClaw), new CA_PitchElevator(liftElevator, false));
  }
}
