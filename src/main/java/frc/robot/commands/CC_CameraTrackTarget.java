// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;

public class CC_CameraTrackTarget extends CommandBase {
  /** Creates a new CC_CameraTrackTarget. */
  Camera m_camera;

  public CC_CameraTrackTarget(Camera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera);
    this.m_camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPan(0);
    m_camera.setTilt(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_camera.isTargetValid()){
      if(!m_camera.isTargetCenter()){
        m_camera.moveCameraToCenter();
      }
      SmartDashboard.putNumber("Y Angle", m_camera.yAngleToTarget());
      SmartDashboard.putNumber("Goal Distance", m_camera.getDistanceToCenterOfHoop());
    } else {
      m_camera.setNeutral();
    }
    SmartDashboard.putBoolean("Target Aligned", m_camera.isTargetValid() && m_camera.isPanInTolerance());
    SmartDashboard.putNumber("Target Error", m_camera.errorToTargetPan());

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
