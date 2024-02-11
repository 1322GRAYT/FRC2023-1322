package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;


public class CT_Music  extends CommandBase  {
    public void execute() {
        Orchestra orca =new Orchestra();
        orca.addInstrument(new TalonFX(Constants.TILT_MOTOR));
        //orca.addInstrument(new TalonFX(Constants.SHOOTER_MOTOR_0));
        //orca.addInstrument(new TalonFX(Constants.SHOOTER_MOTOR_1));
        //orca.addInstrument(new TalonFX(Constants.SHOOTER_PRESHOOT));
        orca.loadMusic("test.chrp");
        orca.play();
        orca.clearInstruments();
        orca = null;
    }
    
}
