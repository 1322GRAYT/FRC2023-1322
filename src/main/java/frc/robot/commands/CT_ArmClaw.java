// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmClawSubsystem.controlState;
import frc.robot.utils.Debounce;
import frc.robot.utils.RFSLIB;
import frc.robot.Constants;
import frc.robot.calibrations.K_LIFT;
import frc.robot.subsystems.ArmClawSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CT_ArmClaw extends CommandBase {
  private ArmClawSubsystem armClawSubsystem;
  private XboxController auxStick;
  private Timer delayTmr;

  private Debounce clawTrigger;

  //private double liftPwr;
  private int dPadPos;

  /** Creates a new CT_LiftRobot. */
  public CT_ArmClaw(ArmClawSubsystem armclawSubsystem, XboxController auxStick) {
    this.armClawSubsystem = armclawSubsystem;
    this.auxStick = auxStick;
    //liftPwr = 0;
    delayTmr = new Timer();
    addRequirements(armclawSubsystem);
    clawTrigger = new Debounce(Constants.DEBOUNCE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot is Armed!");
    armClawSubsystem.setArmControlState(controlState.Back);
    delayTmr.stop();
    delayTmr.reset();
  }

  private boolean downPress(int dir) {
    if (dir<=10 && dir>=350) return true;
    return false;
  }

  private boolean upPress(int dir) {
    if (dir<=190 && dir>=170) return true;
    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*
    basic flow:
    read inputs:
        dpadPosition
        lift power(left Y axis)
        claw power(right X axis)
        check the state based on the dpadPos

        move based on the list power
        //rotate claw on the basis of claw power --- iff the state of claw is not auto
        So the claw needs to pivot as well.....so make that button X


    */
    

    dPadPos = auxStick.getPOV();
    
    double liftPower = RFSLIB.ApplyDeadBand_Scaled(-auxStick.getLeftY(), K_LIFT.KeLIFT_r_CntlrDeadBandThrsh, 1.0);
    double clawPower = RFSLIB.ApplyDeadBand_Scaled(-auxStick.getRightX(), K_LIFT.KeLIFT_r_CntlrDeadBandThrsh, 1.0);
    switch (armClawSubsystem.getArmControlState()) {
      case Back: {
        if (upPress(dPadPos)) armClawSubsystem.setArmControlState(controlState.Mid);
        break;
      }
      case Front: {
        if ( downPress(dPadPos)) armClawSubsystem.setArmControlState(controlState.Mid);
        break;
      }
      case Mid: {
        if ( upPress(dPadPos)) armClawSubsystem.setArmControlState(controlState.Front);
        if ( downPress(dPadPos)) armClawSubsystem.setArmControlState(controlState.Back);
        break;
      }
      default: break;
    }
    armClawSubsystem.setArmPosition();

    if (clawTrigger.checkPress(auxStick.getRightBumper())) armClawSubsystem.clawToggle();

    armClawSubsystem.runArmAtPower(liftPower);
    armClawSubsystem.rotateClaw(clawPower);

    SmartDashboard.putBoolean("LiftSwFront: ",  armClawSubsystem.detectTrackLimitFront());
    SmartDashboard.putBoolean("LiftSwMid: ",    armClawSubsystem.detectTrackMidTrigger());
    SmartDashboard.putBoolean("LiftSwBack: ",   armClawSubsystem.detectTrackLimitRear());
    SmartDashboard.putNumber("ClawRotatePosition", armClawSubsystem.getClawPosition());
    SmartDashboard.putString("ArmControlState: ",    armClawSubsystem.getArmControlState().toString());
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
