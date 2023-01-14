// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.SwerveDrivetrain;

public class CA_SwerveLongDistEncdr extends CommandBase {
  private SwerveDrivetrain swerveDrivetrain;
  private double mtrPwrCmnd;
  private double tgtDistInches;
  private boolean isDirctnRwd;
  private int caddyIndexA;
  private int caddyIndexB;
  private double zeroEncdrCntA;
  private double zeroEncdrCntB;
  private double distTravelledA;
  private double distTravelledB;
  private double rotation;
  private Translation2d translation;


  /** Creates a new CA_SwerveLongDistEncdr. */
  public CA_SwerveLongDistEncdr(SwerveDrivetrain swerveDrivetrain, double mtrPwrCmnd, double tgtDistInches, boolean isDirctnRwd) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.mtrPwrCmnd = mtrPwrCmnd;
    this.tgtDistInches = tgtDistInches;
    this.isDirctnRwd = isDirctnRwd;
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    caddyIndexA = 0; 
    caddyIndexB = 1;
    zeroEncdrCntA = swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexA);
    zeroEncdrCntB = swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexB);

    if (this.mtrPwrCmnd < 0)
      this.mtrPwrCmnd = 0;
    if (this.mtrPwrCmnd > 1)
      this.mtrPwrCmnd = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distTravelledA = swerveDrivetrain.getDrvDistTravelled(caddyIndexA, zeroEncdrCntA);
    distTravelledB = swerveDrivetrain.getDrvDistTravelled(caddyIndexB, zeroEncdrCntB);

      double yAxis =  0.0;
      double xAxis =  this.mtrPwrCmnd;
      double rAxis =  0.0;

      if (!isDirctnRwd) {
        xAxis = -xAxis;
      }

      translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
      rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
      swerveDrivetrain.drive(translation, rotation, false, true);

    if (K_SWRV.KeSWRV_b_DebugEnbl == true) {
      SmartDashboard.putNumber("Auto EncdrCnt ZeroA: ",   (zeroEncdrCntA));
      SmartDashboard.putNumber("Auto EncdrCnt ZeroB: ",   (zeroEncdrCntB));
      SmartDashboard.putNumber("Auto EncdrCnt CurrA: ",   (swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexA)));
      SmartDashboard.putNumber("Auto EncdrCnt CurrB: ",   (swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexB)));
      SmartDashboard.putNumber("Auto DsrdDistTrav: ",     (tgtDistInches));
      SmartDashboard.putNumber("Auto DistTrav A: ",       (distTravelledA));
      SmartDashboard.putNumber("Auto DistTrav B: ",       (distTravelledB));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.stopSwerveDrvMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMetA = Math.abs(distTravelledA) >= tgtDistInches;
    boolean condMetB = Math.abs(distTravelledB) >= tgtDistInches;
    return (condMetA && condMetB);
  }
}
