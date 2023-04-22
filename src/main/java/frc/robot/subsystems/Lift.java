// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Lift extends SubsystemBase {
  private double yawPower = 0;
  private final int INTAKE_PITCH = 10000;
  private final int DELIVERY_PITCH = 25000;
  private double pitchSetPoint = 0;
  private ControlMode pitchMode = ControlMode.PercentOutput;

  private PWM clawYaw = new PWM(Constants.CLAW_YAW_SERVO);
  private WPI_TalonFX clawPitch = new WPI_TalonFX(Constants.CLAW_PITCH);

  public Lift() {
    yawLimiter = new SlewRateLimiter(0.5);
  }

  public double getYawPower() {
    return yawPower;
  }

  public double getPitchSetPoint() {
    return pitchSetPoint;
  }

  public ControlMode getPiControlMode() {
    return pitchMode;
  }

  private SlewRateLimiter yawLimiter;

  public void setYawPower(double yawPower) {
    pitchMode = ControlMode.PercentOutput;
    this.yawPower = yawLimiter.calculate(yawPower);
  }

  public void setPitchPower(double pitchSetPoint) {
    this.pitchSetPoint = pitchSetPoint;
  }

  public void gotoDelivery() {
    pitchMode = ControlMode.MotionMagic;
    pitchSetPoint = DELIVERY_PITCH;
  }

  public void gotoIntake() {
    pitchMode = ControlMode.MotionMagic;
    pitchSetPoint = INTAKE_PITCH;
  }

  // Control Cycles

  private void ControlPitch() {
    clawPitch.set(pitchMode, pitchSetPoint);

  }

  private void ControlYaw() {
    clawYaw.setSpeed(yawPower);

  }

  public void init_periodic() {
  }

  @Override
  public void periodic() {
    ControlPitch();
    ControlYaw();
  }

  @Override
  public void simulationPeriodic() {
  }
}
