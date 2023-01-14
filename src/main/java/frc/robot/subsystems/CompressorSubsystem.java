// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_COMP;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {


private Compressor phCompressor;
private double pressureCompressor;
private double currentCompressor;
private boolean displayDashboardData = true; 


  /** Creates a new CompressorSubsystem. */
  public CompressorSubsystem() {
    phCompressor = new Compressor(Constants.PNEU_HUB_CAN, PneumaticsModuleType.REVPH);
    phCompressor.enableAnalog(90, 120);
    pressureCompressor = 0;
    currentCompressor = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCompressorSensors();
    if (K_COMP.KeCOMP_b_DebugEnbl == true) {
      printCompressorPressure();
    }
  }



  private void updateCompressorSensors() {
    pressureCompressor = phCompressor.getPressure();
    currentCompressor = phCompressor.getCurrent();
  }

  private double getCompressorPressure() {
    return pressureCompressor;
  }

  private double getCompressorCurrent() {
    return currentCompressor;
  }

  private void printCompressorPressure() {
    SmartDashboard.putNumber("Compressor Pressure: ",  getCompressorPressure());
    SmartDashboard.putNumber("Compressor Current: ",   getCompressorCurrent());
  }



}







