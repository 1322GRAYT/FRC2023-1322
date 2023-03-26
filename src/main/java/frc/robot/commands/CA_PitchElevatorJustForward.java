// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftElevator;

public class CA_PitchElevatorJustForward extends CommandBase {
    private static final double POWER = 0.5;
    private LiftElevator liftElevator;
    private boolean out;

    /** Creates a new CA_PitchElevatorJustForward. */
    public CA_PitchElevatorJustForward(LiftElevator liftElevator, boolean out) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.liftElevator = liftElevator;
        this.out = out;
        addRequirements(liftElevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        liftElevator.setPitchPercentPower(out ? -POWER : POWER);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        liftElevator.setPitchPercentPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (liftElevator.getPitchOutSwitch()) || (liftElevator.getPitchInSwitch());
    }
}