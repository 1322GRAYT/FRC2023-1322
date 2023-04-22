package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
    private AHRS gyro;

    public Gyro(Port port) {
        this.gyro = new AHRS(port);
        System.out.println("Starting GYRO");
        gyro.calibrate();
        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (Exception e) {
            }
            zeroGyro();
        }).start();
        System.out.println("GYRO: done");
    }

    /* Gyro */
    public void zeroGyro() {
        gyro.reset();
    }

    private double optimizeGyro(double degrees) {
        return (360 + (degrees % 360)) % 360;
    }

    public Rotation2d getYaw() {
        double pitch = gyro.getPitch();
        double roll = gyro.getRoll();
        double rawYaw = gyro.getYaw();
        SmartDashboard.putNumber("Roll Gyro: ", roll);
        SmartDashboard.putNumber("Pitch Gyro: ", pitch);
        SmartDashboard.putNumber("Yaw Gyro: ", rawYaw);
        double yaw = optimizeGyro(rawYaw);
        return Constants.INVERT_GYRO ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getPitchOptimized() {
        return optimizeGyro(getPitch());
    }

    public double getRollOptimized() {
        return optimizeGyro(getRoll());
    }

    public double getGyroAngleDegrees() {
        return getYaw().getDegrees();
    }

    public double getGyroAngleRadians() {
        return getYaw().getRadians();
    }
}
