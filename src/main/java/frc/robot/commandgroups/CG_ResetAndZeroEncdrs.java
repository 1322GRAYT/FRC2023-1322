// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.CC_TimeDly;
import frc.robot.commands.CC_SwerveResetDrvEncdrs;
import frc.robot.commands.CC_SwerveResetRotEncdrs;
import frc.robot.commands.CC_SwerveZeroGyro;
import frc.robot.commands.CC_SwerveZeroRotEncdrs;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_ResetAndZeroEncdrs extends SequentialCommandGroup {
  /** Creates a new CG_DrvBack. */
  public CG_ResetAndZeroEncdrs(SwerveDrivetrain swerveDrivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      (new CC_SwerveResetRotEncdrs(swerveDrivetrain)),
      (new CC_SwerveResetDrvEncdrs(swerveDrivetrain)),
      (new CC_SwerveZeroGyro(swerveDrivetrain)),
      (new CC_TimeDly(0.100)),
      (new CC_SwerveZeroRotEncdrs(swerveDrivetrain)),
      (new CC_TimeDly(0.150))
    );
    System.out.println("CG_ResetAndZeroEncdrs Autonomous Invoked.");
  }
}
