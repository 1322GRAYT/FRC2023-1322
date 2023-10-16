package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CT_SwerveDriveMotors extends CommandBase {

    private static final double DEADBAND = 0.15;

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveDrivetrain swerveSubsystem;
    private XboxController driverController;
    private XboxController auxController;
    
    public CT_SwerveDriveMotors (SwerveDrivetrain s_Swerve, XboxController driverController, XboxController auxController, boolean fieldRelative, boolean openLoop) {
        this.swerveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        this.driverController = driverController;
        this.auxController = auxController;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        s_Swerve.resetSwerveDriveEncoders();
        s_Swerve.resetSwerveRotateEncoders();
    }

    public int auxButtonPressed() {
        
        if (auxController.getAButton()) return Constants.BUTTON_A;
        if (auxController.getBButton()) return Constants.BUTTON_B;
        if (auxController.getXButton()) return Constants.BUTTON_X;
        if (auxController.getYButton()) return Constants.BUTTON_Y;
        return 0;
    }

    private int _curerent_button=0;
    @Override
    public void execute() {
        //int button = auxButtonPressed();
        //SmartDashboard.putString("Current Motor: ", Constants.BUTTONS[button]);
        double yAxis = -driverController.getLeftY();
        double xAxis = -driverController.getLeftX();
        double rAxis = -driverController.getRightX();

        // double xAxis = -controller.getRawAxis(1);  // Field-Oriented Operator Robot X-Axis Input
        // double yAxis =  controller.getRawAxis(0);  // Field-Oriented Operator Robot Y-Axis Input
        // double rAxis = -controller.getRawAxis(4);

        // make it curve instead of linear
        xAxis=xAxis*xAxis*xAxis;
        yAxis=yAxis*yAxis*yAxis;
        rAxis=rAxis*rAxis*rAxis;
        
        
        /* Deadbands */
        yAxis = ApplyDeadBand_Scaled(yAxis, DEADBAND, 1.0);
        xAxis = ApplyDeadBand_Scaled(xAxis, DEADBAND, 1.0);
        rAxis = ApplyDeadBand_Scaled(rAxis, DEADBAND, 1.0);

        //xAxis = xLimiter.calculate(xAxis);
        //yAxis = yLimiter.calculate(yAxis);
        //rAxis = rLimiter.calculate(rAxis);



        double max_speed =  Constants.SwerveDrivetrain.MAX_SPEED;

        double max_angular_velocity = Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        if (driverController.getRightBumper()) {
            max_speed *= Constants.SwerveDrivetrain.SLOW_SPEED_REDUCTION;
            max_angular_velocity *= Constants.SwerveDrivetrain.SLOW_SPEED_REDUCTION;
        }
        // What the Operator Considers X-Y Axes is Different than Actual Robot Field Orientation
        translation = new Translation2d(yAxis, xAxis).times(max_speed);
        rotation = rAxis * max_angular_velocity;
        swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
    }

    public double ApplyDeadBand_Scaled( double power, double deadBand, double powerLimit) {
		if (power > -deadBand && power < deadBand) return 0.0;
		double sign = (power>0)?1:((power<0)?-1:0);
		if (power > powerLimit || power < - powerLimit) return powerLimit*sign;
		return ((power - sign*deadBand)/(powerLimit - deadBand))*powerLimit;
	}
}
