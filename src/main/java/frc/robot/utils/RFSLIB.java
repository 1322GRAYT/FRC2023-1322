package frc.robot.utils;

//import frc.robot.utils.RFSLIB;

public class RFSLIB /* extends SubsystemBase */ {
	/*********************************/
	/* Dead-Banding Functions */
	/*********************************/
	public static double ApplyDeadBand_Scaled( double power, double deadBand, double powerLimit) {
		if (power > -deadBand && power < deadBand) return 0.0;
		double sign = (power>0)?1:((power<0)?-1:0);
		if (power > powerLimit || power < - powerLimit) return powerLimit*sign;
		return ((power - sign*deadBand)/(powerLimit - deadBand))*powerLimit;
	}

}
