// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class CG_DriveBackOrig extends SequentialCommandGroup {

    private double power = 1.5;
    private double time = 2.0;

    public CG_DriveBackOrig(SwerveDrivetrain drive, LiftClaw liftClaw, LiftElevator liftElevator) {
        addCommands(
                new InstantCommand(() -> {drive.resetSwerveDriveEncoders();drive.resetSwerveRotateEncoders();}),
                new CA_Elevator(liftElevator, true),
                new CA_PitchElevator(liftElevator, true),
                new CA_ToggleClaw(liftClaw),
                new CA_PitchElevator(liftElevator, false),
                new ParallelCommandGroup(
                    new CA_ToggleClaw(liftClaw),
                    new RunCommand(() -> drive.drive(
                        new Translation2d(-power, 0.0), 0, false, true),
                        drive).withTimeout(time)
                ),
                new InstantCommand(() -> drive.stopSwerveDriveMotors()));
    }
}