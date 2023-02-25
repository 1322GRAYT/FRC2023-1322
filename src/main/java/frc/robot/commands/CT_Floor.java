// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utils.Debounce;
import frc.robot.utils.RFSLIB;
import frc.robot.Constants;
import frc.robot.calibrations.K_LIFT;
import frc.robot.subsystems.*;
import frc.robot.subsystems.FloorSubsystem.GrabState;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CT_Floor extends CommandBase {
  private FloorSubsystem floorSubsystem;
  private XboxController auxStick;
  private Timer delayTmr;

  private double liftPwr;
  Debounce rightTrigger,xbutton;


  /** Creates a new CT_LiftRobot. */
  public CT_Floor(FloorSubsystem floorSubsystem, XboxController auxStick) {
    this.floorSubsystem = floorSubsystem;
    this.auxStick = auxStick;
    liftPwr = 0;
    delayTmr = new Timer();
    addRequirements(floorSubsystem);
    rightTrigger = new Debounce(Constants.DEBOUNCE);
    xbutton = new Debounce(Constants.DEBOUNCE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot Lift is Armed!");
    delayTmr.stop();
    delayTmr.reset();
  }




  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    //  there is a grab/ungrab, a lift and drop and go back
    if (rightTrigger.checkPress(auxStick.getXButton())) {
      floorSubsystem.grabToggle();
      //try to grab
    }
    if (xbutton.checkPress(auxStick.getXButton())) {
      //lift, drop and go back
      floorSubsystem.liftDropReset();
    }
    SmartDashboard.putBoolean("LiftSwFront: ",  floorSubsystem.detectTrackLimitFront());
    SmartDashboard.putBoolean("LiftSwBack: ",   floorSubsystem.detectTrackLimitRear());
    SmartDashboard.putString("GrabState ",    ((floorSubsystem.getState() == GrabState.Open)?"Open":"Close"));
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
