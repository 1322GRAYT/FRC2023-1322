// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LiftElevator.pitchState;
import frc.robot.calibrations.ControlSettings;
import frc.robot.subsystems.LiftElevator;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CT_LiftElevator extends CommandBase {
  private LiftElevator liftElevator;
  Supplier<Double> elevatorControl, pitchControl;

  /** Creates a new CT_LiftRobot. */
  public CT_LiftElevator(LiftElevator liftElevator, Supplier<Double> elevatorControl, Supplier<Double> pitchControl) {
    this.liftElevator = liftElevator;
    this.elevatorControl = elevatorControl;
    this.pitchControl = pitchControl;
    addRequirements(liftElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }


  private long debounce_A_time=System.currentTimeMillis(); 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Don't interupt Motion Magic unless movement in joystick
    // TODO: Add factor to allow for stoppoing of arm.
    if (Math.abs(elevatorControl.get()) - ControlSettings.AUX_STICK_DEADBAND > 0 || liftElevator.getElevatorState() == ControlMode.PercentOutput){
      liftElevator.setElevatorPercentPower(elevatorControl.get());
    }


    if (Math.abs(pitchControl.get()) - ControlSettings.AUX_STICK_DEADBAND > 0 || liftElevator.getPitchState() == ControlMode.PercentOutput){
      liftElevator.setPitchPercentPower(pitchControl.get());
    }


   /* if (auxStick.getAButtonPressed()) {
      liftElevator.setElevatorPosition(-64000);
    }
    */
    // Toggle between Positions
    // if (auxStick.getLeftBumperPressed() && liftElevator.getElevatorPitch() != pitchState.Back){
    //   liftElevator.setElevatorPitch(liftElevator.getElevatorPitch().previous());
    // }
    
    // if (auxStick.getRightBumperPressed() && liftElevator.getElevatorPitch() != pitchState.Front){
    //   liftElevator.setElevatorPitch(liftElevator.getElevatorPitch().next());
    // }
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
