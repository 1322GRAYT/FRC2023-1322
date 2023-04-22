
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class CG_YeetCubeRamp extends SequentialCommandGroup {

    //no ramp was 1.5, 2.0
    private double power = 2.5;
    private double time = 1.8;

    public CG_YeetCubeRamp(SwerveDrivetrain drive, Gyro gyro, LiftClaw liftClaw, LiftElevator liftElevator) {
        addCommands(
            new CA_Elevator(liftElevator, true).withTimeout(3),
            new CA_PitchElevatorJustForward(liftElevator, true),
            new CA_YeetCube(liftClaw),
            //new CA_ToggleClaw(liftClaw),
            //new CA_PitchElevator(liftElevator, false),
            new RunCommand(() -> drive.drive(new Translation2d(0.0,-power), 0, false, true),drive).withTimeout(time),
            new AutoBalance(drive, gyro)
            );
    }
}