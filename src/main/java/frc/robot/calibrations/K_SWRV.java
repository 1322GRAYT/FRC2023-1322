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
public class K_SWRV {



  /**************************************************/
  /*  Swerve Drive Configuration Calibrations       */
 	/**************************************************/	 	

	  /** KeSWRV_b_DebugEnbl: Swerve Drive System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeSWRV_b_DebugEnbl = true;



  /**************************************************/
  /*  Swerve Drive General System Calibrations      */
 	/**************************************************/	 	

	  /** KeSWRV_r_CntlrDeadBandThrsh: Swerve Drive System: Normalized
     * Power Dead-Band Threshold that must be met before a X-Box Controller
     * Joystick Input is recognized.  If the Absolute value of the Input
     * is below the Threshold it will be ignored (i.e. set to Zero).
     */
    public static final double KeSWRV_r_CntlrDeadBandThrsh = 0.3;

    

  /**************************************************/
  /*  Swerve Drive Design Parameters                */
 	/**************************************************/	 	



  /**************************************************/
  /*  Swerve Rotation-System Mechanical Parameters  */
 	/**************************************************/	 	




  /*******************************************************/
  /*  Swerve Drive-System Design Mechanical Parameters   */
 	/*******************************************************/	 	
	




  /****************************************************/
  /*  Swerve Drive Rotation Control PID Coefficients  */
 	/****************************************************/	 	
    


  /**************************************************/
  /*  Swerve Drive Drive Control PID Coefficients   */
 	/**************************************************/	 	

 



  /******************************************/
  /*  Swerve Drive Current Driver Limits    */
 	/******************************************/	 	
 



  /********************************************/
  /*  Rotation Zero Offset Learn Algorithm    */
 	/********************************************/	 	



  /******************************************************/
  /*  Swerve Drive Control: Closed Loop Drive Control   */
 	/******************************************************/	 	

   /** KeSWRV_Deg_CL_DrvErrTgtDB_Rot: Swerve Drive System - 
     * Closed Loop Control Drive Control Error Target Deadband
     * Below which Closed-Loop will be considered Complete for
     * Rotational control (CW/CCW).
     */    
    public static final double KeSWRV_Deg_CL_DrvErrTgtDB_Rot = 1;
    
   /** KeSWRV_t_CL_DrvSyncThrshRot: Swerve Drive System - 
     * Closed Loop Control Drive Control Sync Time for
     * Rotational Control.
     */    
    public static final double KeSWRV_t_CL_DrvSyncThrshRot = 0.100;

    public static final double KeSWRV_r_CL_FdFwdPwr_Rot = 0.4;
    public static final double KeSWRV_k_CL_PropGx_Rot   = 0.1;
    public static final double KeSWRV_k_CL_IntglGx_Rot  = 0.001;
    public static final double KeSWRV_k_CL_DerivGx_Rot  = 0;


}
