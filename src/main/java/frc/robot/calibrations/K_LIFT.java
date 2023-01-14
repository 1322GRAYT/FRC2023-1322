/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Add your docs here.
 */
public class K_LIFT {



  /**************************************************/
  /*  Robot Lift Control Configuration Calibrations */
 	/**************************************************/	 	

	  /** KeLIFT_b_DebugEnbl: Lift System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeLIFT_b_DebugEnbl = true;



  /***************************************************/
  /*  Robot Lift Control General System Calibrations */
 	/***************************************************/
   
 	  /** KeLIFT_r_CntlrDeadBandThrsh: Lift System: Normalized
     * Power Dead-Band Threshold that must be met before a X-Box Controller
     * Joystick Input is recognized.  If the Absolute value of the Input
     * is below the Threshold it will be ignored (i.e. set to Zero).
     */
    public static final double KeLIFT_r_CntlrDeadBandThrsh = 0.2;

 	  /** KeLIFT_t_RaiseTrackRearSwDlyTme: Lift System: Delay time
     * after tripping the rear track switch before the command
     * to raise the front track section will be issued.  To
     * allow the swinging of the robot to dampen before raising
     * the track so the front of track does not extend above the
     * next bar.
     */
    public static final double KeLIFT_t_RaiseTrackRearSwDlyTme = 1.0;

  

  /*****************************************/
  /*  Robot Lift Control PID Coefficients  */
 	/*****************************************/	 	
    
	  /** KeLIFT_K_Prop: Lift System Proporational 
     * Control Gain. 
     */
    public static final double KeLIFT_K_Prop = 0.1;


	  /** KeLIFT_K_Intgl: Lift System Integral Control 
     * Gain. 
     */
    public static final double KeLIFT_K_Intgl = 0.001;


	  /** KeLIFT_K_Deriv: Lift System Derivative Control 
     * Gain. 
     */
    public static final double KeLIFT_K_Deriv = 0.0;


	  /** KeLIFT_K_FdFwd: Lift System Feed Fowrward Control 
     * Gain. 
     */
    public static final double KeLIFT_K_FdFwd = 0.001;


	  /** KeLIFT_r_IntglErrMaxEnbl: Lift System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeLIFT_r_IntglErrMaxEnbl = 0.1;



}
