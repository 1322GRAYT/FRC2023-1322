// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CA_SwerveRotRel extends CommandBase {
  private final SwerveDrivetrain swerveDrivetrain;
  private Camera camera;
  private boolean stopAtTgt; // Stop if Camera Target is Detected While Rotating
  private double Xe_r_PwrFFTerm;
  private double Xe_r_RotPwr;
  private double Xe_Deg_HdgAngInit;
  private double Xe_Deg_RotAngDsrd;
  private double Xe_Deg_RotAngTgt;
  private double Xe_Deg_RotAngErr;
  private boolean Xe_b_WithinDB;
  private boolean Xe_b_SyncTmrStrtd = false;
  private Timer Xe_t_SyncTmr = new Timer();
  private double rotation;
  private Translation2d translation;


  /** Creates a new CA_SwerveRotAbs. */
  public CA_SwerveRotRel(SwerveDrivetrain swerveDrivetrain, Camera camera, double tgtRotPstn, boolean stopAtTgt) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.camera = camera;
    this.stopAtTgt = stopAtTgt;
    Xe_Deg_HdgAngInit = swerveDrivetrain.getGyroAngleDegrees();
    Xe_Deg_RotAngDsrd = tgtRotPstn;
    Xe_Deg_RotAngTgt = Xe_Deg_HdgAngInit + Xe_Deg_RotAngDsrd;

    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Xe_t_SyncTmr.stop();
    Xe_t_SyncTmr.reset();
    Xe_b_SyncTmrStrtd = false;

    Xe_Deg_RotAngErr = Xe_Deg_RotAngTgt - swerveDrivetrain.getGyroAngleDegrees();
    Xe_r_PwrFFTerm = K_SWRV.KeSWRV_r_CL_FdFwdPwr_Rot;
    if (Xe_Deg_RotAngErr < 0.0)
      {Xe_r_PwrFFTerm = -Xe_r_PwrFFTerm;}

    swerveDrivetrain.resetRotPID();
    swerveDrivetrain.configPID_RotCtrl(Xe_Deg_RotAngTgt);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Xe_Deg_RotAngErr = Xe_Deg_RotAngTgt - swerveDrivetrain.getGyroAngleDegrees();
    Xe_r_RotPwr = swerveDrivetrain.PID_RotCtrl(Xe_Deg_RotAngErr);
    swerveDrivetrain.setDRV_r_PID_RotPowCorr(Xe_r_RotPwr);  

    double yAxis = 0.0;
    double xAxis = 0.0;
    double rAxis = Xe_r_PwrFFTerm + Xe_r_RotPwr;
    
    translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
    rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
    swerveDrivetrain.drive(translation, rotation, false, true);

    Xe_b_WithinDB = swerveDrivetrain.getRotAtSetpoint();        
    if (Xe_b_WithinDB) {
        if (Xe_b_SyncTmrStrtd == false) {
            Xe_t_SyncTmr.start();
            Xe_b_SyncTmrStrtd = true;
        }
    } else {
        Xe_t_SyncTmr.stop();
        Xe_t_SyncTmr.reset();
        Xe_b_SyncTmrStrtd = false;
    }

    SmartDashboard.putNumber("Rot Cmd Err " ,   Xe_Deg_RotAngErr);
    SmartDashboard.putNumber("Rot Cmd FF " ,    Xe_r_PwrFFTerm);
    SmartDashboard.putNumber("Rot Cmd CL " ,    Xe_r_RotPwr);
    SmartDashboard.putNumber("Rot Cmd Pwr " ,   rAxis);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.stopSwerveDrvMotors();
    Xe_t_SyncTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean endCondA = (Xe_t_SyncTmr.hasElapsed(K_SWRV.KeSWRV_t_CL_DrvSyncThrshRot));
    boolean endCondB = (stopAtTgt && camera.isTargetValid());
    return (endCondA | endCondB);
  }
}
