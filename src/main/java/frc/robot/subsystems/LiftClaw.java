// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftClaw extends SubsystemBase {

  // Internal Definitions
  public enum ClawState {
    Open, Closed;
  }
  private ClawState clawState = ClawState.Closed;
  private double yawPower = 0;
  private double pitchSetPoint = 0;
  private ControlMode pitchMode = ControlMode.PercentOutput;

  // Resource Definitions
  private PWM clawYaw = new PWM(Constants.CLAW_YAW_SERVO);
  private WPI_TalonFX clawPitch = new WPI_TalonFX(Constants.CLAW_PITCH);
  private DoubleSolenoid clawGrab = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_CLAW_0, Constants.PNEUMATIC_CLAW_1);
  // private AnalogInput clawAngle = new AnalogInput(Constants.CLAW_ANGLE_SENSOR);
  

  /**
   * Constructor
   */
  public LiftClaw() {}

  // Getter Interfaces

  public ClawState getClawState() {
    return clawState;
  }

  public double getYawPower() {
    return yawPower;
  }

  public double getPitchSetPoint() {
    return pitchSetPoint;
  }

  // public double getClawAngle(){
  //   return clawAngle.getVoltage();
  // }
  
  // Setter Interfaces

  public void setClawState(ClawState clawState) {
    this.clawState = clawState;
  }

  public void setYawPower(double yawPower) {
    pitchMode = ControlMode.PercentOutput;
    this.yawPower = yawPower;
  }

  public void setPitchSetPoint(double pitchSetPoint) {
    this.pitchSetPoint = pitchSetPoint;
  }

  // Control Cycles

  private void ControlPitch(){
    clawPitch.set(pitchMode, pitchSetPoint);

  }

  private void ControlYaw(){
    clawYaw.setSpeed(yawPower);

  }

  private void ControlClaw(){
    Value set = Value.kOff;
    switch(clawState){
      case Closed:
        set = Value.kReverse;
        break;
      case Open:
        set = Value.kForward;
        break;     
    }

    clawGrab.set(set);
  }

  public void init_periodic() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ControlPitch();
    ControlYaw();
    ControlClaw();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
