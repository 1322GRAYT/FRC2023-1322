// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.calibrations;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/** Add your docs here. 
 * TODO: What is the point of this?
*/
public class configArmLift {
    public TalonFXConfiguration _fx = new TalonFXConfiguration();

    public configArmLift() {
        // TODO: Verify configuration from json to here
        _fx.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        _fx.primaryPID.selectedFeedbackCoefficient = 1.0;
        _fx.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        _fx.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        _fx.openloopRamp = 1.023000;
        _fx.closedloopRamp = 1.705000;
        _fx.peakOutputForward = 1.0;
        _fx.peakOutputReverse = -1.0;
        _fx.neutralDeadband = 0.15;
        _fx.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        _fx.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        _fx.slot0.kP = 0.5;
        _fx.slot0.kI = 0;
        _fx.slot0.kD = 0;
        _fx.slot0.kF = 0.57;
        _fx.slot0.integralZone = 0;
        _fx.slot0.allowableClosedloopError = 750;
        _fx.slot0.maxIntegralAccumulator = 0;
        _fx.slot0.closedLoopPeakOutput = 1.0;
        _fx.slot0.closedLoopPeriod = 1;
        _fx.motionCruiseVelocity = 13500;
        _fx.motionAcceleration = 8000;
        _fx.clearPositionOnLimitF = false;
        _fx.clearPositionOnLimitR = true;
        _fx.motorCommutation = MotorCommutation.Trapezoidal;
        _fx.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    }

    
}
