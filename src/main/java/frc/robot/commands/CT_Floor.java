// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.FloorPickup.GrabState;
import frc.robot.utils.Direction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CT_Floor extends CommandBase {
  private FloorPickup floorSubsystem;
  private XboxController auxStick;


  

  /** Creates a new CT_LiftRobot. */
  public CT_Floor(FloorPickup floorSubsystem, XboxController auxStick) {
    this.floorSubsystem = floorSubsystem;
    this.auxStick = auxStick;
    addRequirements(floorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(Direction.valueOf(auxStick.getPOV())){
      case UP:
      floorSubsystem.setFloorMotorPower(0.2);
      break;
      case DOWN:
      floorSubsystem.setFloorMotorPower(-0.2);
      break;
      case LEFT:
      floorSubsystem.setGrabState(GrabState.Open);
      break;
      case RIGHT:
      floorSubsystem.setGrabState(GrabState.Closed);
      break;
      case NONE:
      default:
      floorSubsystem.setFloorMotorPower(0);
      break;
    }
    floorSubsystem.setConeMotorPower(auxStick.getLeftTriggerAxis()-auxStick.getRightTriggerAxis());
    floorSubsystem.setConeRotatePower(auxStick.getBackButton()?1:0);
    floorSubsystem.setConeGrab(auxStick.getStartButton());

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
