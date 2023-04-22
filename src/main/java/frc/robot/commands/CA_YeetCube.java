// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CA_YeetCube extends InstantCommand {
    private Claw claw;

    public CA_YeetCube(Claw claw) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(claw);
        this.claw = claw;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        claw.setIntakeMotorPower(-1.0);
        //Timer.delay(1);
        //liftClaw.setIntakeMotorPower(0);
    }
}