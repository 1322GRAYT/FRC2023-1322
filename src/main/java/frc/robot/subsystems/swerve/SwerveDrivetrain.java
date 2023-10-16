package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

  private SwerveDriveOdometry _swerveOdometry;
  private SwerveModule[] _swerveModules;
  private AHRS _gyro;
  private Field2d _field;



  /* Rotation Control PID */
/*   private PIDController rotPID;

  private double rotPID_PowOutMin;
  private double rotPID_PowOutMax;
 
 */
  public SwerveDrivetrain() {

    System.out.println(System.currentTimeMillis() + " Starting SwerveDriveTrain");
  
    // gyroscope calibration
    System.out.print(System.currentTimeMillis() + " SwerveDriveTrain -- calibrating gyro - ");
    this._gyro = new AHRS(Constants.SwerveDrivetrain.GYRO_ID);
    this._gyro.calibrate();
    this._gyro.reset();

    System.out.println(System.currentTimeMillis() + " Completed -- gyro");

    // set up swerve modules
    System.out.println("Setting up modules");
    this._swerveModules = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveDrivetrain.Modules[0]),
        new SwerveModule(1, Constants.SwerveDrivetrain.Modules[1]),
        new SwerveModule(2, Constants.SwerveDrivetrain.Modules[2]),
        new SwerveModule(3, Constants.SwerveDrivetrain.Modules[3])
    };
    
    System.out.println(System.currentTimeMillis() + " SwerveDriveTrain -- SwerveModules");


    this._swerveOdometry = new SwerveDriveOdometry(
        Constants.SwerveDrivetrain.SWERVE_KINEMATICS,
        this.getYaw(),
        getSwerveModulePositions()
        );

    this._field = new Field2d();

    System.out.println(System.currentTimeMillis() + " Starting SwerveDriveTrain -- odometry");

    /* Rotation Control PID */
   /*  rotPID = new PIDController(
      Constants.SwerveDrivetrain.ROTATE_kP, 
      Constants.SwerveDrivetrain.ROTATE_kI,
      Constants.SwerveDrivetrain.ROTATE_kD
      );
    // rotPID_PowCorr = 0;
    rotPID_PowOutMin = -1;
    rotPID_PowOutMax = 1; */

    System.out.println(System.currentTimeMillis() + " Starting SwerveDriveTrain  -- rotPID");

    //dashboard();
  }



  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[_swerveModules.length];
    for (int i = 0; i < _swerveModules.length; i++) {
      swerveModulePositions[i] = _swerveModules[i].getPosition();
    }
    return swerveModulePositions;
  }

  public void print() {

    // output module states
    for (SwerveModule module: _swerveModules) {
      SmartDashboard.putNumber("Swerve Module " + module._moduleNumber + " Position: ", module.getDriveEncoderPosition());
      SmartDashboard.putNumber("Swerve Module" + module._moduleNumber + " Rotation: ", module.getRotateEncoderPosition());
    }

    SmartDashboard.putNumber("Swerve Field X-Coord: ", _field.getRobotPose().getX());
    SmartDashboard.putNumber("Swerve Field Y-Coord: ", _field.getRobotPose().getY());
    SmartDashboard.putNumber("Swerve Yaw: ", getYaw().getDegrees());
    SmartDashboard.putNumber("Swerve Roll: ", getRoll().getDegrees());
    SmartDashboard.putNumber("Swerve Pitch: ", getPitch().getDegrees());

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            this.getYaw())
            : 
            new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation)
        );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);

    for (SwerveModule mod : this._swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod._moduleNumber], isOpenLoop);
    }
  }

  /* Gyro */
  public void zeroGyro() {
    _gyro.reset();
  }
 
  private double optimizeGyro(double degrees) {
    return (360+(degrees%360)) % 360;
  }

  public Rotation2d getYaw() {
    double yaw = optimizeGyro(_gyro.getYaw());
    return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360.0 - yaw) : Rotation2d.fromDegrees(yaw);
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(_gyro.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(_gyro.getRoll());
  }

/*   public double getGyroAngleDegrees() {
    return this.getYaw().getDegrees();
  }
 */
  /* Odometry */

  public Pose2d getPose() {
    return this._swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    this._swerveOdometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
  }

  public void resetSwerveDriveEncoders() {
    for (SwerveModule module: _swerveModules) {
      module.resetDriveEncoderPosition();
    }
  }

  public void resetSwerveRotateEncoders() {
    for (SwerveModule module: _swerveModules) {
      module.findZeroRotation();
    }
  }

  public void stopSwerveDriveMotors() {
    for (SwerveModule module: _swerveModules) {
      module.stopDriveMotor();
    }
  }

  public void stopSwerveRotateMotors() {
    for (SwerveModule module: _swerveModules) {
      module.stopRotateMotor();
    }
  }

/*   public void stopSwerveCaddyDrvMotor(int motorIndex) {
    _swerveModules[motorIndex].stopDriveMotor();
  } */

  /**
   * Method: getDrvInchesPerEncdrCnts - Calculates the nominal
   * Linear Distance that the Wheel would travel forward/
   * backward if the Drive Wheel Encoder has/would have
   * registered the given number of encoder counts.
   * 
   * @param: Encoder Counts (cnts)
   * @return: Linear Wheel Distance (inches)
   */
/*   public double getDrvInchesPerEncdrCnts(double encoderCounts) {
    double encoderRevolutions;
    double wheelRevolutions;
    double wheelDistanceInches;

    encoderRevolutions = encoderCounts / (double) Constants.SwerveDrivetrain.DRIVE_CNTS_PER_REV;
    wheelRevolutions = encoderRevolutions / (double) Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO;
    wheelDistanceInches = wheelRevolutions * (double) Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE;

    
    return (wheelDistanceInches);
  } */

  /**
   * Method: getDrvEncdrCntsPerInches - Calculates the nominal
   * number of Drive encoder counts that would be registered
   * if the Drive Wheel traveled forward/backward the
   * desired distance given (inches).
   * 
   * @param: Desired Distance (inches)
   * @return: Encoder Counts (cnts)
   */
/*   public double getDrvEncdrCntsPerInches(double wheelDistInches) {
    double wheelRevs;
    double encdrRevs;
    double encdrCnts;

    wheelRevs = wheelDistInches / (double) Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE;
    encdrRevs = wheelRevs * (double) Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO;
    encdrCnts = encdrRevs * (double) Constants.SwerveDrivetrain.DRIVE_CNTS_PER_REV;

    return (Math.round(encdrCnts));
  } */

/*   public double getDrvDistTravelled(int mtrIdx, double zeroPstnRefCnts) {
    double drvEncdrCntDelt;
    drvEncdrCntDelt = Math.round(_swerveModules[mtrIdx].getDriveEncoderPosition() - zeroPstnRefCnts);
    return (getDrvInchesPerEncdrCnts(drvEncdrCntDelt));
  } */

/*   public double getDrvCaddyEncdrPstn(int mtrIdx) {
    return Math.round(_swerveModules[mtrIdx].getDriveEncoderPosition());  
  } */

/*   public void dashboard() {
    //ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    //tab.add(this);
    //tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
    //tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
    //tab.add("Field X-Coord ", this._field.getRobotPose().getX());
    //tab.add("Field Y-Coord ", this._field.getRobotPose().getY());

  } */

  public void init_periodic() {
    // This method will be called once per robot periodic/autonmous session at
    // initiation
  }

  @Override
  public void periodic() {
    print();
    //this.swerveOdometry.update(this.getYaw(), this.getStates());
    _swerveOdometry.update(getYaw(), getSwerveModulePositions());
    _field.setRobotPose(this._swerveOdometry.getPoseMeters());
  }

}
