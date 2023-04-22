package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;
  // private AHRS gyro;
  private Gyro gyro;
  private Field2d field;

  /* Rotation Control PID */
  private PIDController rotPID;
  // private double rotPID_PowCorr;
  private double rotPID_PowerOutMinimum;
  private double rotPID_PowerOutMaximum;
  // private double rotPID_RotAngCmnd;

  public SwerveDrivetrain(Gyro gyro) {
      this.gyro = gyro;

      swerveModules = new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveDrivetrain.Module0),
          new SwerveModule(1, Constants.SwerveDrivetrain.Module1),
          new SwerveModule(2, Constants.SwerveDrivetrain.Module2),
          new SwerveModule(3, Constants.SwerveDrivetrain.Module3)
      };
      System.out.println("SwerveDriveTrain -- SwerveModules");

      swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS,
          gyro.getYaw(),
          getSwerveModulePositions());

      field = new Field2d();

      System.out.println("Starting SwerveDriveTrain -- odometry");

    /* Rotation Control PID */
    rotPID = new PIDController(Constants.KeSWRV_k_CL_PropGx_Rot, Constants.KeSWRV_k_CL_IntglGx_Rot,
    Constants.KeSWRV_k_CL_DerivGx_Rot);
    // rotPID_PowCorr = 0;
    rotPID_PowerOutMinimum = -1;
    rotPID_PowerOutMaximum = 1;
    // rotPID_RotAngCmnd = getYaw().getDegrees();
    System.out.println("Starting SwerveDriveTrain  -- rotPID");

    dashboard();
  }

  /*
   * public SwerveModule getSwerveModule(int i) {
   * return swerveModules[i];
   * }
   */

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
    }
    return swerveModulePositions;
  }

  public void print() {

    swerveModules[0].getCanCoder();
    swerveModules[1].getCanCoder();
    swerveModules[2].getCanCoder();
    swerveModules[3].getCanCoder();

    // SmartDashboard.putNumber("Field X-Coord: ", field.getRobotPose().getX());
    // SmartDashboard.putNumber("Field Y-Coord: ", field.getRobotPose().getY());
    // SmartDashboard.putNumber("Yaw: ", getYaw().getDegrees());

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            gyro.getYaw())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }







  /*
   * public void resetToAbsolute() {
   * swerveModules[0].resetToAbsolute();
   * swerveModules[1].resetToAbsolute();
   * swerveModules[2].resetToAbsolute();
   * swerveModules[3].resetToAbsolute();
   * }
   */

  /* Odometry */

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    // swerveOdometry.resetPosition(pose, getSwerveModulePositions(),
    // pose.getRotation());
    swerveOdometry.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    // swerveOdometry.resetPosition(pose, getSwerveModulePositions(),
    // getYaw());
    swerveOdometry.resetPosition(gyro.getYaw(), getSwerveModulePositions(), pose);
  }

  /* Module States */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrivetrain.MAX_SPEED);
    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetSwerveDriveEncoders() {
    for (SwerveModule module : swerveModules) {
      module.resetDriveEncoderPosition();
    }
  }

  public void resetSwerveRotateEncoders() {
    for (SwerveModule module : swerveModules) {
      module.resetRotaeEncoderPosition();
    }
  }

  public void stopSwerveDriveMotors() {
    for (SwerveModule module : swerveModules) {
      module.stopDriveMotor();
    }
  }

  public void stopSwerveRotMotors() {
    for (SwerveModule module : swerveModules) {
      module.stopRotateMotor();
    }
  }

  public void stopSwerveCaddyDrvMotor(int motorIndex) {
    swerveModules[motorIndex].stopDriveMotor();
  }

  /**
   * Method: getDrvInchesPerEncdrCnts - Calculates the nominal
   * Linear Distance that the Wheel would travel forward/
   * backward if the Drive Wheel Encoder has/would have
   * registered the given number of encoder counts.
   * 
   * @param: Encoder Counts (cnts)
   * @return: Linear Wheel Distance (inches)
   */
  public double getDrvInchesPerEncdrCnts(double encoderCounts) {
    double encoderRevolutions;
    double wheelRevolutions;
    double wheelDistanceInches;

    encoderRevolutions = encoderCounts / (double) Constants.SwerveDrivetrain.DRIVE_COUNTS_PER_REVOLUTION;
    wheelRevolutions = encoderRevolutions / (double) Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO;
    wheelDistanceInches = wheelRevolutions * (double) Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE;

    return (wheelDistanceInches);
  }

  /**
   * Method: getDrvEncdrCntsPerInches - Calculates the nominal
   * number of Drive encoder counts that would be registered
   * if the Drive Wheel traveled forward/backward the
   * desired distance given (inches).
   * 
   * @param: Desired Distance (inches)
   * @return: Encoder Counts (cnts)
   */
  public double getDrvEncdrCntsPerInches(double wheelDistInches) {
    double wheelRevs;
    double encdrRevs;
    double encdrCnts;

    wheelRevs = wheelDistInches / (double) Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE;
    encdrRevs = wheelRevs * (double) Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO;
    encdrCnts = encdrRevs * (double) Constants.SwerveDrivetrain.DRIVE_COUNTS_PER_REVOLUTION;

    return (Math.round(encdrCnts));
  }

  public double getDrvDistTravelled(int mtrIdx, double zeroPstnRefCnts) {
    double drvEncdrCntDelt;
    drvEncdrCntDelt = Math.round(swerveModules[mtrIdx].getDriveEncoderCurrentPosition() - zeroPstnRefCnts);
    return (getDrvInchesPerEncdrCnts(drvEncdrCntDelt));
  }

  public double getDrvCaddyEncdrPstn(int mtrIdx) {
    return Math.round(swerveModules[mtrIdx].getDriveEncoderCurrentPosition());
  }

  public void dashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    tab.add(this);
    tab.addNumber("Gyro Angle ???", gyro::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
    tab.addNumber("Gyro Angle (GRAPH) ???", gyro::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
    tab.add("Field X-Coord ", field.getRobotPose().getX());
    tab.add("Field Y-Coord ", field.getRobotPose().getY());
    // SmartDashboard.putData(field);
    // SmartDashboard.putData("ANGLE PID", data);
    // SmartDashboard.putData("DRIVE PID", data);
  }

  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
  }

  @Override
  public void periodic() {
    print();
    // swerveOdometry.update(getYaw(), getStates());
    swerveOdometry.update(gyro.getYaw(), getSwerveModulePositions());
    field.setRobotPose(swerveOdometry.getPoseMeters());
  }

  /*****************************************************************/
  /** Chassis Rotation Control PID */
  /*****************************************************************/

  public void setDRV_r_PID_RotPowCorr(double LeDRV_r_PID_RotPowCorr) {
    // rotPID_PowCorr = LeDRV_r_PID_RotPowCorr;
  }

  /**
   * Method: PID_RotCtrl - Drive System: Control Drive Rotational Control
   * with a PID controller using the Gyro as a Reference.
   * 
   * @param roteteError The error term (from the input device, generally
   *                    gyroscope) for rotation
   *                    and heading correction.
   */
  public double PID_RotateControl(double roteteError) {
    double rotateErrorCorrected;
    rotateErrorCorrected = rotPID.calculate(roteteError);
    rotateErrorCorrected = MathUtil.clamp(rotateErrorCorrected, rotPID_PowerOutMinimum, rotPID_PowerOutMaximum);
    return (rotateErrorCorrected);
  }

  public void configPID_RotCtrl(double Le_Deg_RotAngTgt) {
    // Rotation PID (has continuous input)
    rotPID.setPID(Constants.KeSWRV_k_CL_PropGx_Rot, Constants.KeSWRV_k_CL_IntglGx_Rot, Constants.KeSWRV_k_CL_DerivGx_Rot);
    setRotWraparoundInputRange(0, 360);
    setRotSetpoint(Le_Deg_RotAngTgt);
    setRotTolerance(5, 5);
    setRotOutputRange(-1, 1);
  }

  /*************************************************/
  /* Drive ROT PID Methods for configuring PID */
  /*************************************************/

  public void setRotSetpoint(double setpoint) {
    rotPID.setSetpoint(setpoint);
  }

  public void setRotTolerance(double positionTolerance, double velocityTolerance) {
    rotPID.setTolerance(positionTolerance, velocityTolerance);
  }

  public void setRotWraparoundInputRange(double min, double max) {
    rotPID.enableContinuousInput(min, max);
  }

  public void setRotAccumulationRange(double min, double max) {
    rotPID.setIntegratorRange(min, max);
  }

  public void setRotOutputRange(double minOutput, double maxOutput) {
    rotPID_PowerOutMinimum = minOutput;
    rotPID_PowerOutMaximum = maxOutput;
  }

  public double getRotErrorDerivative() {
    return rotPID.getVelocityError();
  }

  public boolean getRotAtSetpoint() {
    return rotPID.atSetpoint();
  }

  /**
   * Clear all I accumulation, disable continuous input, and set all
   * setpoints to 0.
   */
  public void resetRotPID() {
    // clear I accumulation
    rotPID.reset();

    // reset to noncontinuous input
    rotPID.disableContinuousInput();

    // set all setpoints to 0
    rotPID.setSetpoint(0);

    // set all I accumulation ranges to defaults
    rotPID.setIntegratorRange(-0.5, 0.5);

    rotPID.setTolerance(0.05, Double.POSITIVE_INFINITY);

    rotPID_PowerOutMinimum = -1;
    rotPID_PowerOutMaximum = 1;
  }

  /*************************************************/
  /* Subsystem Instrumenation Display */
  /*************************************************/

  // private void updateSmartDash() {
  /* Print to SmartDashboard */
  /*
   * SmartDashboard.putNumber("Gyro Snsd Ang " , VeDRV_Deg_NAV_SnsdAng);
   * SmartDashboard.putNumber("Gyro Dsrd Ang " , VeDRV_Deg_NAV_DsrdAng);
   * 
   * SmartDashboard.putNumber("Pwr PID Drv " , VeDRV_r_PID_DrvPowCorr);
   * SmartDashboard.putNumber("PID SetPt Drv " , VeDRV_Cnt_PID_DrvPstnCmnd);
   * SmartDashboard.putNumber("Pwr PID Rot " , VeDRV_r_PID_RotPowCorr);
   * SmartDashboard.putNumber("PID SetPt Rot " , VeDRV_Deg_PID_RotAngCmnd);
   * 
   * for (int i = 0; i < DrvMap.NumOfMtrs; i++) {
   * SmartDashboard.putNumber("Drv Encdr Cnts " + i , VaDRV_Cnt_DrvEncdrPstn[i]);
   * SmartDashboard.putNumber("Drv Encdr Zero Pstn " + i ,
   * VaDRV_Cnt_DrvEncdrZeroPstn[i]);
   * SmartDashboard.putNumber("Drvn Encdr Dist " + i , getDrvDistance(i));
   * 
   * }
   */
  // }

}
