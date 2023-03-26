package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CT_SwerveDrive extends CommandBase {

    private static final double DEADBAND = 0.25;

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveDrivetrain swerveSubsystem;
    private XboxController controller;
    
    public CT_SwerveDrive (SwerveDrivetrain s_Swerve, XboxController controller, boolean fieldRelative, boolean openLoop) {
        this.swerveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        s_Swerve.resetSwerveDriveEncoders();
        s_Swerve.resetSwerveRotateEncoders();
    }

    @Override
    public void execute() {
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        // double xAxis = -controller.getRawAxis(1);  // Field-Oriented Operator Robot X-Axis Input
        // double yAxis =  controller.getRawAxis(0);  // Field-Oriented Operator Robot Y-Axis Input
        // double rAxis = -controller.getRawAxis(4);
        
        /* Deadbands */
        yAxis = ApplyDeadBand_Scaled(yAxis, DEADBAND, 1.0);
        xAxis = ApplyDeadBand_Scaled(xAxis, DEADBAND, 1.0);
        rAxis = ApplyDeadBand_Scaled(rAxis, DEADBAND, 1.0);

        //xAxis = xLimiter.calculate(xAxis);
        //yAxis = yLimiter.calculate(yAxis);
        //rAxis = rLimiter.calculate(rAxis);



        double max_speed =  Constants.SwerveDrivetrain.MAX_SPEED;

        if (controller.getRightBumper()) {
            max_speed = max_speed/2;
        }
        // What the Operator Considers X-Y Axes is Different than Actual Robot Field Orientation
        translation = new Translation2d(yAxis, xAxis).times(max_speed);
        rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
    }

    public double ApplyDeadBand_Scaled( double power, double deadBand, double powerLimit) {
		if (power > -deadBand && power < deadBand) return 0.0;
		double sign = (power>0)?1:((power<0)?-1:0);
		if (power > powerLimit || power < - powerLimit) return powerLimit*sign;
		return ((power - sign*deadBand)/(powerLimit - deadBand))*powerLimit;
	}
}
