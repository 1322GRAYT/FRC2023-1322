// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.CC_SwerveResetOdometry;
import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_SwerveResetOdometry extends SequentialCommandGroup {
  /** Creates a new CG_SwerveResetOdometry. */
  public CG_SwerveResetOdometry(SwerveDrivetrain swerveDrivetrain

  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      (new CC_SwerveResetOdometry(swerveDrivetrain, swerveDrivetrain.getPose()))
    );
  }
}
