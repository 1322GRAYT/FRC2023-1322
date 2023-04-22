// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class BalanceOnRamp extends SequentialCommandGroup {

    //no ramp was 1.5, 2.0
    private double power = 2.5;
    private double time = 1.8;

    public BalanceOnRamp(SwerveDrivetrain drive, Gyro gyro, Claw claw, LiftElevator liftElevator) {
        addCommands(
            new CA_Elevator(liftElevator, true).withTimeout(3),
            new CA_PitchElevator(liftElevator, true),
            new CA_ToggleClaw(claw),
            new CA_PitchElevator(liftElevator, false),
            new ParallelCommandGroup(
                new CA_ToggleClaw(claw),
                new RunCommand(() -> drive.drive(
                    new Translation2d(0.0,-power), 0, false, true),drive).withTimeout(time)
            ),
            //new InstantCommand(() -> drive.stopSwerveDriveMotors()),
            new AutoBalance(drive,gyro)
            );
    }
}