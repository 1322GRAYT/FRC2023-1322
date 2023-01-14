package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.RFSLIB;
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
    
    private SwerveDrivetrain s_Swerve;
    private XboxController controller;

    public CT_SwerveDrive (SwerveDrivetrain s_Swerve, XboxController controller, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        // double yAxis = -controller.getLeftY();
        // double xAxis = -controller.getLeftX();
        // double rAxis = -controller.getRightX();

        double xAxis = -controller.getRawAxis(1);  // Field-Oriented Operator Robot X-Axis Input
        double yAxis =  controller.getRawAxis(0);  // Field-Oriented Operator Robot Y-Axis Input
        double rAxis = -controller.getRawAxis(4);
        
        /* Deadbands */
        yAxis = RFSLIB.ApplyDB_Scld(yAxis, DEADBAND, 1.0);
        xAxis = RFSLIB.ApplyDB_Scld(xAxis, DEADBAND, 1.0);
        rAxis = RFSLIB.ApplyDB_Scld(rAxis, DEADBAND, 1.0);

        // What the Operator Considers X-Y Axes is Different than Actual Robot Field Orientation
        translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
        rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
