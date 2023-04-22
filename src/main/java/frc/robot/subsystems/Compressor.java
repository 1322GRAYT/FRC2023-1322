// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {

  private static Compressor instance;
  private edu.wpi.first.wpilibj.Compressor phCompressor;
  private double pressureCompressor;
  //private double currentCompressor;
  // private boolean displayDashboardData = true;

  public static Compressor getInstance() {
    if (instance == null) {
      instance = new Compressor();
    }
    return instance;
  }
  /** Creates a new CompressorSubsystem. */
  private Compressor() {
    phCompressor = new edu.wpi.first.wpilibj.Compressor(Constants.PNEUMATIC_COMPRESSOR, PneumaticsModuleType.REVPH);
    phCompressor.enableAnalog(90, 120);
    pressureCompressor = 0;
    //currentCompressor = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCompressorSensors();
    if (Constants.PNEUMATIC_COMPRESSOR_DEBUG_ENABLE) {
      printCompressorPressure();
    }
  }

  private void updateCompressorSensors() {
    pressureCompressor = phCompressor.getPressure();
    //currentCompressor = phCompressor.getCurrent();
  }

  public double getCompressorPressure() {
    return pressureCompressor;
  }

  //private double getCompressorCurrent() {
  //  return currentCompressor;
 // }
  

  private void printCompressorPressure() {
    SmartDashboard.putNumber("Compressor Pressure: ", getCompressorPressure());
    //SmartDashboard.putNumber("Compressor Current: ", getCompressorCurrent());
  }

}
