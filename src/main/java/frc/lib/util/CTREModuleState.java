package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param currentAngle Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double currentAngle, double newAngle) {
        double lowerBound;
        double upperBound;
        
        // this makes loweroffset in the range -360 <= lowerOffset <= 360
        double lowerOffset = currentAngle % 360;

        if (lowerOffset >= 0) {
            lowerBound = currentAngle - lowerOffset;   // lowerbound should be 0 in this case if currentangle is 0 <= currentAngle <= 360
            upperBound = currentAngle + (360 - lowerOffset); //upperbound should be 360 inb this case if currentangle is 0 <= currentAngle <= 360
        } else {
            upperBound = currentAngle - lowerOffset;  //upperbound should be 0 in this case if currentangle is -360 <= currentAngle <= 0
            lowerBound = currentAngle - (360 + lowerOffset); //lowerbound should be -360 in this case if currentangle is -360 <= currentAngle <= 0
        }


        while (newAngle < lowerBound) {
            newAngle += 360;
        }   

        while (newAngle > upperBound) {
            newAngle -= 360;
        }

        if (newAngle - currentAngle > 180) {
            newAngle -= 360;
        } else if (newAngle - currentAngle < -180) {
            newAngle += 360;
        }
        
        return newAngle;
    }
}
