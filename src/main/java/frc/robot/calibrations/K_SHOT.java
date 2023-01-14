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
public class K_SHOT {


  /**************************************************/
  /*  Shooter System Configuration Calibrations     */
 	/**************************************************/	 	

	  /** KeSHOT_b_DebugEnbl: Shooter System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeSHOT_b_DebugEnbl = true;



  /**************************************************/
  /*  Shooter System General System Calibrations    */
 	/**************************************************/	 	

    /** KeSHOT_t_IntakeArmRaiseDlyShoot: Shooter System - Delay
     * Time Prior to Commanding to Raise the Intake Arms after
     * the Balls are no longer detected at the neither Advance
     * Position 1 Nor Position 2 when in the Ball Shoot Command.
     * Allows for some Debounce Filtering.
     */
    public static final double KeSHOT_t_IntakeArmRaiseDlyShoot = 0.100;



  /**************************************************/
  /*  Shooter System Launch Calibrations            */
 	/**************************************************/


	  /** KeSHOT_n_AtTgtDB: Deadband Speed around the
     * Commanded Target that it will be assumed that
     * the Shooter Motor is at the Commanded Target.
     */
    public static final double KeSHOT_n_AtTgtDB = 200;

	  /** KeSHOT_n_AtTgtDB_Hystersis: Deadband Speed 
     * Hystersis to determine when to exit the Deadband
     * once entered to that Intake Motor jitter does not
     * occur.
     */
    public static final double KeSHOT_n_AtTgtDB_Hystersis = 500;



	  /** KeSHOT_n_TgtLaunchCmdLoGoal: Shooter System Control 
     * Commmanded Shooter Speed during Tele-Op - Low Goal.
     */
    public static final double KeSHOT_n_TgtLaunchCmdLoGoal = 4250;


	  /** KeSHOT_n_TgtLaunchCmdLoGoal: Shooter System Control 
     * Commmanded Shooter Speed during Autonomous - Low Goal. 
     */
    public static final double KeSHOT_n_TgtLaunchCmdLoGoalAuto = 4250;

	  /** KeSHOT_n_TgtLaunchCmdHiGoalCloseAuto: Shooter System Control
     * Commmanded Shooter Speed during Autonomous - High Goal Close
     * Distance. 
     */
    public static final double KeSHOT_n_TgtLaunchCmdHiGoalCloseAuto = 8500;

	  /** KeSHOT_n_TgtLaunchCmdHiGoalCloseAuto: Shooter System Control
     * Commmanded Shooter Speed during Autonomous - High Goal Mid
     * Distance. 
     */
    public static final double KeSHOT_n_TgtLaunchCmdHiGoalMidAuto = 10000;

	  /** KeSHOT_n_TgtLaunchCmdHiGoalCloseAuto: Shooter System Control
     * Commmanded Shooter Speed during Autonomous - High Goal Mid
     * Distance. 
     */
    public static final double KeSHOT_n_TgtLaunchCmdHiGoalFarAuto = 15000;


	  /** KtSHOT_n_ShooterSpdCmdTgtHi: Shooter System Target
     * Motor Speed to as a function of distance to the goal
     * for the high goal got Tele-Op. 
     */
    public static final float[] KtSHOT_n_ShooterSpdCmdTgtHi =  
    {  7500,    //  40  -  1 
       7700,    //  50  -  2
       8400,    //  55  -  3
       8700,    //  60  -  4
       8850,    //  65  -  5
       9000,    //  70  -  6
       9150,    //  75  -  7
       9400,    //  80  -  8
      10200,    //  85  -  9
      11700,    //  90  - 10
      13700,    // 100  - 11
      15200};   // 110  - 12

      
	  /** KnSHOT_l_ShooterDistAxis: Shooter System Command
     * Target Distance Axis for High Goal for Tele-Op. 
     */
    public static final float KnSHOT_l_ShooterDistAxis[] =
    {   40,  //  1
        50,  //  2
        55,  //  3
        60,  //  4
        65,  //  5
        70,  //  6
        75,  //  7
        80,  //  8
        85,  //  9
        90,  // 10
       100,  // 11
       110}; // 12


	  /** KtSHOT_n_ShooterSpdCmdTgtHiAuto: Shooter System Target
     * Motor Speed to as a function of distance to the goal
     * for the high goal for Autonomous. 
     */
    public static final float[] KtSHOT_n_ShooterSpdCmdTgtHiAuto =  
    {  8500,    //  60  -  1
       9000,    //  65  -  2
       9500,    //  70  -  3
      10000,    //  75  -  4
      11000,    //  80  -  5
      13000,    //  85  -  6
      15000,    //  90  -  7
      18000,    // 100  -  8
      20000};   // 110  -  9

	  /** KnSHOT_l_ShooterDistAxisAuto: Shooter System Command
     * Target Distance Axis for High Goal for Autonomous. 
     */
    public static final float KnSHOT_l_ShooterDistAxisAuto[] =
    {   60,  //  1
        65,  //  2
        70,  //  3
        75,  //  4
        80,  //  5
        85,  //  6
        90,  //  7
       100,  //  8
       110}; //  9



    /** KeSHOT_t_PostLaunchRunTime: The amount of time
     * to keep the Shooter Motors Running after the balls
     * have left the Ball Feed Advance Positions (assuming
     * the Controller Button is still not being held). 
     */
    public static final double KeSHOT_t_PostLaunchRunTime = 2.0;



    
  /*************************************************/
  /*  Shooter System Control PID Coefficients      */
 	/*************************************************/	 	
    
	  /** KeSHOT_K_Prop: Shooter System Control Proporational
     * Control Gain. 
     */
    public static final double KeSHOT_K_Prop = 0.070; // 0.062 Base Gain


	  /** KeSHOT_K_Intgl: Shooter System Control Integral 
     * Control Gain. 
     */
    public static final double KeSHOT_K_Intgl = 0.0;


	  /** KeSHOT_K_Deriv: Shooter System Control
     * Derivative Control Gain. 
     */
    public static final double KeSHOT_K_Deriv = 0.0;


	  /** KeSHOT_K_FdFwd: Shooter System Control Feed
     * Fowrward Control Gain. 
     */
    public static final double KeSHOT_K_FdFwd = 0.0500;  // 0.0475 Base Gain

	  /** KeSHOT_r_IntglErrMaxEnbl: Shooter System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral,correction will applied.  Error Signal must
     * be within band (+/- postive value). (revs)
     */
    public static final double KeSHOT_r_IntglErrMaxEnbl = 0.1;



}
