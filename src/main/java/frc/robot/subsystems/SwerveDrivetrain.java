package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.calibrations.K_SWRV;
import frc.robot.utils.swerve.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;
  private AHRS gyro;
  private Field2d field;

  /* Rotation Control PID */
  private PIDController rotPID;
  // private double rotPID_PowCorr;
  private double rotPID_PowOutMin;
  private double rotPID_PowOutMax;
  // private double rotPID_RotAngCmnd;

  // Creates a new TrapezoidProfile
  // Profile will have a max vel of 3 meters per second
  // Profile will have a max acceleration of 6 meters per second squared
  // Profile will end stationary at 5 meters
  // Profile will start stationary at zero position
  /*
   * TrapezoidProfile rotPID_RotProfile = new TrapezoidProfile(new
   * TrapezoidProfile.Constraints(3, 6),
   * new TrapezoidProfile.State(3, 0),
   * new TrapezoidProfile.State(0, 0));
   */

  public SwerveDrivetrain() {
    this.gyro = new AHRS(Constants.SwerveDrivetrain.GYRO_ID);
    this.gyro.calibrate();
    this.zeroGyro();

    this.swerveModules = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveDrivetrain.Mod0.constants),
        new SwerveModule(1, Constants.SwerveDrivetrain.Mod1.constants),
        new SwerveModule(2, Constants.SwerveDrivetrain.Mod2.constants),
        new SwerveModule(3, Constants.SwerveDrivetrain.Mod3.constants)
    };

    this.swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS,
        this.getYaw(),
        getSwerveModulePositions());

    this.field = new Field2d();

    /* Rotation Control PID */
    rotPID = new PIDController(K_SWRV.KeSWRV_k_CL_PropGx_Rot, K_SWRV.KeSWRV_k_CL_IntglGx_Rot,
        K_SWRV.KeSWRV_k_CL_DerivGx_Rot);
    // rotPID_PowCorr = 0;
    rotPID_PowOutMin = -1;
    rotPID_PowOutMax = 1;
    // rotPID_RotAngCmnd = this.getYaw().getDegrees();

    dashboard();
  }

  public SwerveModule getSwerveModule(int i) {
    return swerveModules[i];
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModulePositions[i] = swerveModules[i].geSwerveModulePosition();
    }
    return swerveModulePositions;
  }

  public void print() {

    this.swerveModules[0].getCanCoder();
    this.swerveModules[1].getCanCoder();
    this.swerveModules[2].getCanCoder();
    this.swerveModules[3].getCanCoder();

    SmartDashboard.putNumber("Field X-Coord: ", this.field.getRobotPose().getX());
    SmartDashboard.putNumber("Field Y-Coord: ", this.field.getRobotPose().getY());
    SmartDashboard.putNumber("Fixed Gyro: ", this.getYaw().getDegrees());

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            this.getYaw())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);

    for (SwerveModule mod : this.swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Gyro */
  public void zeroGyro() {
    this.gyro.reset();
  }

  private double optimizeGyro(double degrees) {
    // 0 < degrees < 360
    if ((degrees > 0.0) && (degrees < 360.0)) {
      return degrees;
    } else {
      int m = (int) Math.floor(degrees / 360.0);
      double optimizedDegrees = degrees - (m * 360.0);
      return Math.abs(optimizedDegrees);
    }
  }

  public Rotation2d getYaw() {
    double pitch = this.gyro.getPitch();
    double roll = this.gyro.getRoll();
    double rawYaw = this.gyro.getYaw();
    SmartDashboard.putNumber("Roll Gyro: ", roll);
    SmartDashboard.putNumber("Pitch Gyro: ", pitch);
    SmartDashboard.putNumber("Yaw Gyro: ", rawYaw);
    double yaw = optimizeGyro(pitch);
    return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }

  public double getGyroAngleDegrees() {
    return this.getYaw().getDegrees();
  }

  public double getGyroAngleRadians() {
    return this.getYaw().getRadians();
  }

  public void resetToAbsolute() {
    this.swerveModules[0].resetToAbsolute();
    this.swerveModules[1].resetToAbsolute();
    this.swerveModules[2].resetToAbsolute();
    this.swerveModules[3].resetToAbsolute();
  }

  /* Odometry */

  public Pose2d getPose() {
    return this.swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    // this.swerveOdometry.resetPosition(pose, getSwerveModulePositions(),
    // pose.getRotation());
    this.swerveOdometry.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    // this.swerveOdometry.resetPosition(pose, getSwerveModulePositions(),
    // this.getYaw());
    this.swerveOdometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
  }

  // public boolean runUntilZero() {
  // double encoder0 = this.swerveModules[0].getCanCoder().getDegrees() -
  // this.swerveModules[0].getAngleOffset();
  // double encoder1 = this.swerveModules[1].getCanCoder().getDegrees() -
  // this.swerveModules[1].getAngleOffset();
  // double encoder2 = this.swerveModules[2].getCanCoder().getDegrees() -
  // this.swerveModules[2].getAngleOffset();
  // double encoder3 = this.swerveModules[3].getCanCoder().getDegrees() -
  // this.swerveModules[3].getAngleOffset();
  // boolean allAtZero = true;
  // if(encoder0 > 3) {
  // this.swerveModules[0].runAngleByPercent(.2);
  // allAtZero = false;
  // } else {
  // this.swerveModules[0].runAngleByPercent(0);
  // }
  // if(encoder1 > 3) {
  // this.swerveModules[1].runAngleByPercent(.2);
  // allAtZero = false;
  // } else {
  // this.swerveModules[1].runAngleByPercent(0);
  // }
  // if(encoder2 > 3) {
  // this.swerveModules[2].runAngleByPercent(.2);
  // allAtZero = false;
  // } else {
  // this.swerveModules[2].runAngleByPercent(0);
  // }
  // if(encoder3 > 3) {
  // this.swerveModules[3].runAngleByPercent(.2);
  // allAtZero = false;
  // } else {
  // this.swerveModules[3].runAngleByPercent(0);
  // }
  // return allAtZero;
  // }

  /* Module States */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : this.swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrivetrain.MAX_SPEED);
    for (SwerveModule mod : this.swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetSwerveDrvEncdrs() {
    for (int i = 0; i < 4; i++) {
      getSwerveModule(i).resetDrvEncdrPstn();
      ;
    }
  }

  public void resetSwerveRotEncdrs() {
    for (int i = 0; i < 4; i++) {
      getSwerveModule(i).resetRotEncdrPstn();
      ;
    }
  }

  public void zeroSwerveRotEncdrs() {
    for (int i = 0; i < 4; i++) {
      getSwerveModule(i).zeroRotEncdrPstn();
      ;
    }
  }

  public void stopSwerveDrvMotors() {
    for (int i = 0; i < 4; i++) {
      getSwerveModule(i).stopDrvMotor();
    }
  }

  public void stopSwerveRotMotors() {
    for (int i = 0; i < 4; i++) {
      getSwerveModule(i).stopRotMotor();
    }
  }

  public void stopSwerveCaddyDrvMotor(int mtrIdx) {
    getSwerveModule(mtrIdx).stopDrvMotor();
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
  public double getDrvInchesPerEncdrCnts(double encdrCnts) {
    double encdrRevs;
    double wheelRevs;
    double wheelDistInches;

    encdrRevs = encdrCnts / (double) Constants.SwerveDrivetrain.DRIVE_CNTS_PER_REV;
    wheelRevs = encdrRevs / (double) Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO;
    wheelDistInches = wheelRevs * (double) Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE;

    return (wheelDistInches);
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
    encdrCnts = encdrRevs * (double) Constants.SwerveDrivetrain.DRIVE_CNTS_PER_REV;

    return (Math.round(encdrCnts));
  }

  public double getDrvDistTravelled(int mtrIdx, double zeroPstnRefCnts) {
    double drvEncdrCntDelt;
    drvEncdrCntDelt = Math.round(getSwerveModule(mtrIdx).getDrvEncdrCurrentPstn() - zeroPstnRefCnts);
    return (getDrvInchesPerEncdrCnts(drvEncdrCntDelt));
  }

  public double getDrvCaddyEncdrPstn(int mtrIdx) {
    double drvEncdrCnt = Math.round(getSwerveModule(mtrIdx).getDrvEncdrCurrentPstn());
    return drvEncdrCnt;
  }

  public void dashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    tab.add(this);
    tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
    tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
    tab.add("Field X-Coord ", this.field.getRobotPose().getX());
    tab.add("Field Y-Coord ", this.field.getRobotPose().getY());
    SmartDashboard.putData(this.field);
    // SmartDashboard.putData("ANGLE PID", data);
    // SmartDashboard.putData("DRIVE PID", data);
  }

  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
  }

  @Override
  public void periodic() {
    this.print();
    //this.swerveOdometry.update(this.getYaw(), this.getStates());
    this.swerveOdometry.update(getYaw(), getSwerveModulePositions());
    this.field.setRobotPose(this.swerveOdometry.getPoseMeters());
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
   * @param Le_Deg_CL_RotErr The error term (from the input device, generally
   *                         gyroscope) for rotation
   *                         and heading correction.
   */
  public double PID_RotCtrl(double Le_Deg_CL_RotErr) {
    double Le_r_CL_CorrNormPwr;
    Le_r_CL_CorrNormPwr = rotPID.calculate(Le_Deg_CL_RotErr);
    Le_r_CL_CorrNormPwr = MathUtil.clamp(Le_r_CL_CorrNormPwr, rotPID_PowOutMin, rotPID_PowOutMax);
    return (Le_r_CL_CorrNormPwr);
  }

  public void configPID_RotCtrl(double Le_Deg_RotAngTgt) {
    // Rotation PID (has continuous input)
    rotPID.setPID(K_SWRV.KeSWRV_k_CL_PropGx_Rot, K_SWRV.KeSWRV_k_CL_IntglGx_Rot, K_SWRV.KeSWRV_k_CL_DerivGx_Rot);
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
    rotPID_PowOutMin = minOutput;
    rotPID_PowOutMax = maxOutput;
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

    rotPID_PowOutMin = -1;
    rotPID_PowOutMax = 1;
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
