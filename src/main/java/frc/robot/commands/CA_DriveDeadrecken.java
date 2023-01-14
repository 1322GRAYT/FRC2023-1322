// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class CA_DriveDeadrecken extends CommandBase {
  private SwerveDrivetrain drive;
  Translation2d setSpeed;
  private double rSpeed;
  private Pose2d targetPose;
  private Pose2d currentPose;
  Timer playTime;
  private double time;


  /** Creates a new CA_DriveDeadrecken. */
  public CA_DriveDeadrecken(SwerveDrivetrain drive, double xSpeed, double ySpeed, double rSpeed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.rSpeed = rSpeed * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
    this.setSpeed = new Translation2d(xSpeed, ySpeed).times(Constants.SwerveDrivetrain.MAX_SPEED);
    playTime = new Timer();
    this.time = time;
  }

  public CA_DriveDeadrecken(SwerveDrivetrain drive, Translation2d Translate, double rSpeed, Pose2d Pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.rSpeed = rSpeed * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
    this.setSpeed = Translate;
    this.targetPose = Pose;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(this.setSpeed, this.rSpeed, false, true);
    drive.resetOdometry(new Pose2d(0.0,0.0,new Rotation2d(0)));
    playTime.reset();
    playTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this.currentPose = drive.getPose();
    //SmartDashboard.putNumber("Driver X Pose", drive.getPose().getX());
    //SmartDashboard.putNumber("Driver Y Pose", drive.getPose().getY());
    //SmartDashboard.putNumber("x Pose", currentPose.getX());
    //SmartDashboard.putNumber("y Pose", currentPose.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(0,0).times(1), 0, false, true);
    System.out.println("Drives - This is done");
    System.out.println(Math.pow(currentPose.getX() - targetPose.getX(),2) + Math.pow(currentPose.getY() - targetPose.getY(),2));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return playTime.get() > time;
  }
}
