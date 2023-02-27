// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LiftElevator.pitchState;
import frc.robot.calibrations.ControlSettings;
import frc.robot.subsystems.LiftElevator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CT_ArmClaw extends CommandBase {
  private LiftElevator liftElevator;
  private XboxController auxStick;

  /** Creates a new CT_LiftRobot. */
  public CT_ArmClaw(LiftElevator liftElevator, XboxController auxStick) {
    this.liftElevator = liftElevator;
    this.auxStick = auxStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Don't interupt Motion Magic unless movement in joystick
    // TODO: Add factor to allow for stoppoing of arm.
    if (Math.abs(auxStick.getLeftY()) - ControlSettings.AUX_STICK_DEADBAND > 0 || liftElevator.getElevatorState() == ControlMode.PercentOutput){
      liftElevator.setElevatorPercentPower(auxStick.getLeftY());
    }

    // Toggle between Positions
    if (auxStick.getLeftBumperPressed() && liftElevator.getElevatorPitch() != pitchState.Back){
      liftElevator.setElevatorPitch(liftElevator.getElevatorPitch().previous());
    }
    
    if (auxStick.getRightBumperPressed() && liftElevator.getElevatorPitch() != pitchState.Front){
      liftElevator.setElevatorPitch(liftElevator.getElevatorPitch().next());
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
