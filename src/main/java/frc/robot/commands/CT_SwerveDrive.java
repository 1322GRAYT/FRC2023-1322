package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commandgroups.CG_ResetAndZeroEncdrs;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CT_SwerveDrive extends CommandBase {

    private static final double DEADBAND = 0.15;

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveDrivetrain swerveSubsystem;
    private XboxController controller;
    private Debouncer ybutton;
    
    public CT_SwerveDrive (SwerveDrivetrain s_Swerve, XboxController controller, boolean fieldRelative, boolean openLoop) {
        this.swerveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        s_Swerve.resetSwerveDriveEncoders();
        //s_Swerve.resetSwerveRotateEncoders();
        ybutton = new Debouncer(.1,DebounceType.kBoth);
    }

    @Override
    public void execute() {


        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

/*         if (holdHeading && !stopHold && (Math.abs(xAxis) < 0.1) && (Math.abs(yAxis) < 0.1)) {
            swerveSubsystem.holdPosition(swerveSubsystem.getCurrentPose());
            return;
        } else {
            swerveSubsystem.stopHoldposition();
        } */

        // make it curve instead of linear
        xAxis=xAxis*xAxis*xAxis;
        yAxis=yAxis*yAxis*yAxis;
        rAxis=rAxis*rAxis*rAxis;
        
        
        /* Deadbands */
        yAxis = Constants.ApplyDeadBand_Scaled(yAxis, DEADBAND, 1.0);
        xAxis = Constants.ApplyDeadBand_Scaled(xAxis, DEADBAND, 1.0);
        rAxis = Constants.ApplyDeadBand_Scaled(rAxis, DEADBAND, 1.0);


        if (ybutton.calculate(controller.getYButton())) {
            CommandScheduler.getInstance().schedule(new CG_ResetAndZeroEncdrs(swerveSubsystem));
        }


        double max_speed =  Constants.SwerveDrivetrain.MAX_SPEED;

        double max_angular_velocity = Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        if (controller.getRightBumper()) {
            max_speed *= Constants.SwerveDrivetrain.SLOW_SPEED_REDUCTION;
            max_angular_velocity *= Constants.SwerveDrivetrain.SLOW_SPEED_REDUCTION;
        }
        // What the Operator Considers X-Y Axes is Different than Actual Robot Field Orientation
        translation = new Translation2d(yAxis, xAxis).times(max_speed);
        rotation = rAxis * max_angular_velocity;
        swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
    }
}
