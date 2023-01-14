// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Camera extends SubsystemBase {
  final static Servo Tilt = new Servo(1);
  final static Servo Pan = new Servo(2);

  final static double CAMERA_ANGLE_TOLERANCE = 1;

  final static double ANGLE_TILT_ZERO = 0.3;
  final static double ANGLE_TILT_HORIZONTAL = 0.1;
  final static double ANGLE_PAN_ZERO = 0.6925;

  final static double kP_x = 0.0005;
  final static double kP_y = 0.0005;

  private final double kD_x = 0;
  private final double kD_y = 0;

  private final static double MAX_SPEED = 0.08;

  final static double ANGLE_PAN_TOLERANCE = 0.05;

  public enum Targets{GOAL, BLUE_BALL, RED_BALL};

  Targets currentTarget = Targets.GOAL;

  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setTilt(double position) {
    Tilt.set(position);
  }

  public double getTilt(){
    return Tilt.getPosition();
  }

  public void setPan(double position) {
    Pan.set(position);
  }

  public double getPan(){
    return Pan.getPosition();
  }

  public boolean isTargetValid(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
  }

  public double getCameraTargetXAngle() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getCameraTargetYAngle() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  // Pipeline values 0 - Goal, 1 - Blue Ball, 2 - Red Ball
  public void setCameraPipeline(Targets target){
    currentTarget = target;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(currentTarget.ordinal());
  }

  public void setNeutral() {
    setPan(ANGLE_PAN_ZERO);
    setTilt(ANGLE_TILT_ZERO);
  }

  public boolean isTargetCenter() {
    return Math.sqrt(Math.pow(getCameraTargetXAngle(), 2) + Math.pow(getCameraTargetXAngle(), 2)) < CAMERA_ANGLE_TOLERANCE;
  }

  double[] p_error = {0,0};
  
  public void moveCameraToCenter() {
    double[] error = errorToTarget();

    setPan(clipSpeed(error[0]*kP_y +  kD_x * (error[0] - p_error[0]), -MAX_SPEED, MAX_SPEED) + getPan());
    setTilt(clipSpeed(error[1]*kP_y +  kD_y * (error[1] - p_error[1]), -MAX_SPEED, MAX_SPEED) + getTilt());

    p_error = error;
  }

  private double[] errorToTarget(){
    double[] output = {getCameraTargetXAngle(), getCameraTargetYAngle()};
    return output;
  }

  /** errorToTargetPan: Pan Angle Error to the Target.  A postive
   *  error value indicates the center of the target is to the right
   *  of the cross-hair of the shooter. A negative error value
   *  indicate the center of the target is to the left of the
   *  cross-hair of the shooter.
   */
  public double errorToTargetPan(){
    double errorTargetPan = getCameraTargetXAngle();
    return errorTargetPan;
  }

  private double clipSpeed(double value, double min, double max){
    return (value - max > 0 ? max : (min - value > 0 ? min : value));
  }

  public boolean isPanInTolerance(){
    return Math.abs(getPan() - ANGLE_PAN_ZERO) < ANGLE_PAN_TOLERANCE;
  }


  final static double SERVO_ANGULAR_RANGE= 270;
  public double xAngleToTarget(){
    return (getPan() - ANGLE_PAN_ZERO) * SERVO_ANGULAR_RANGE + getCameraTargetXAngle();
  }

  public double yAngleToTarget(){
    return (getTilt() - ANGLE_TILT_HORIZONTAL) * 270 + getCameraTargetYAngle();
  }

  final static double HEIGHT_OF_CAMERA = 51; // inches
  final static double HEIGHT_OF_GOAL = 104; // inches
  final static double RADIUS_OF_GOAL = 24; // inches
  public double getDistanceToCenterOfHoop(){
    return (HEIGHT_OF_GOAL - HEIGHT_OF_CAMERA)/Math.tan(Math.toRadians(yAngleToTarget())) + RADIUS_OF_GOAL;
  }

}