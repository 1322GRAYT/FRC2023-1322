/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDelay extends CommandBase {
  /**
   * Command: CC_TimeDly - Delays for a the amount
   * of time specified in the argument.
   */
  Timer  delayTimer;
  double durationSeconds;  

  public TimeDelay(double durationSeconds) {
    this.durationSeconds = durationSeconds;
    delayTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayTimer.reset();
    delayTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    delayTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (delayTimer.get() > durationSeconds);
  }
}
