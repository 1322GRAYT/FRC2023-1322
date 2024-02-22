package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {
    private double _tX,_tY;
    private int _targetID;
    private NetworkTable limelightTable;
    private boolean _targetMode;

    public CameraSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        _targetMode = false;
    }

    public double tX() {
        return _tX;
    }

    public double tY() {
        return _tY;
    }

    public int targetID() {
        return _targetID;
    }

    public String getTarget() {
        if (_targetID == 0) {
            return "No Target";
        } else {
            return Constants.AprilTags.get(_targetID);
        }
    }

    public void setTargetMode() {
        _targetMode = true;
    }

    public void setDrivingMode() {
        _targetMode = false;
    }


    public void setCameraLightsOn() {
        limelightTable.getEntry("ledMode").setNumber(3);
    }
    public void setCameraLightsOff() {
        limelightTable.getEntry("ledMode").setNumber(1);
    }

    public void setCamModeVision() {
        limelightTable.getEntry("camMode").setNumber(0);
    }
    public void setCamModeDriver() {
        limelightTable.getEntry("camMode").setNumber(1);
    }

    
    @Override
    public void periodic() {
        if (
            (int) limelightTable.getEntry("tv").getDouble(0) > 0
            && _targetMode
            ) {
            _tX = limelightTable.getEntry("tx").getDouble(0);
            _tY = limelightTable.getEntry("ty").getDouble(0);
            _targetID = (int)limelightTable.getEntry("tsid").getDouble(0);
        } else {
            _tX = 0;
            _tY = 0;
            _targetID = 0;
            SmartDashboard.putString("Limelight Detected","No");
        }
    }
}
