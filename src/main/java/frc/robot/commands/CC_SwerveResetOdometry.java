// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;


import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CC_SwerveResetOdometry extends InstantCommand {
  private SwerveDrivetrain swerveDrivetrain;
  private Pose2d pose;


  public CC_SwerveResetOdometry(SwerveDrivetrain swerveDrivetrain, Pose2d pose) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.pose = pose;
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrivetrain.resetOdometry(pose);
  }
}
