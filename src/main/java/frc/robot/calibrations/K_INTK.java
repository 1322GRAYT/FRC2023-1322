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
public class K_INTK {



  /***************************************************/
  /*  Ball Intake Control Configuration Calibrations */
 	/***************************************************/	 	

	  /** KeINTK_b_DebugEnbl: Ball Intake System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeINTK_b_DebugEnbl = true;



  /****************************************************/
  /*  Ball Intake Control General System Calibrations */
 	/****************************************************/	 	

    /** KeINTK_r_IntakeMtrTriggerLvlEnbl: Ball Intake System 
     * Aux Controller Right Trigger Postion (0 to 1) at or
     * above that the Ball Intake System will be enabled. 
     */
    public static final double KeINTK_r_IntakeMtrTriggerLvlEnbl = 0.5;
    
    /** KeINTK_r_IntakeMtrTriggerLvlDsbl: Ball Intake System 
     * Aux Controller Right Trigger Postion (0 to 1) below
     * above that the Ball Intake System will be disabled if
     * it is presently active. 
     */
    public static final double KeINTK_r_IntakeMtrTriggerLvlDsbl = 0.3;


    /** KeINTK_t_IntakeArmDtctTme: Ball Intake System Ball 
     * present minimun detection time for Intake Arms.
     */
    public static final double KeINTK_t_IntakeArmDtctTme = 0.040;


    /** KeINTK_t_IntakeAdv1DtctTme: Ball Intake System Ball 
     * present minimun detection time for Intake Advance
     * Position 1.
     */
    public static final double KeINTK_t_IntakeAdv1DtctTme = 0.020;

    /** KeINTK_t_IntakeAdv1CaptTme: Ball Intake System Ball 
     * present minimun capture time for Intake Advance
     * Position 1.
     */
    public static final double KeINTK_t_IntakeAdv1CaptTme = 0.050;


    /** KeINTK_t_IntakeAdv2DtctTme: Ball Intake System Ball 
     * present minimun detection time for Intake Advance
     * Position 2.
     */
    public static final double KeINTK_t_IntakeAdv2DtctTme = 0.020;

    /** KeINTK_t_IntakeAdv2CaptTme: Ball Intake System Ball 
     * present minimun capture time for Intake Advance
     * Position 2.
     */
    public static final double KeINTK_t_IntakeAdv2CaptTme = 0.050;


    /** KeINTK_t_IntakeFullOverrideTmeOut: Ball Intake
     * System Ball Full Override Time Out.  If the Operator
     * is Requesting to Run the Intake Motors without Shooting
     * but Two Balls are detected in the Intake System, time
     * out the Intake and Advance Motors after this time. 
     */
    public static final double KeINTK_t_IntakeFullOverrideTmeOut = 0.500;

    /** KeINTK_t_IntakeArmRaiseLowerDlyCmd: Ball Intake System Delay
     * Time To Keep Command Active before terminating when requesting
     * to Raise or Lower the Intake Arms.
     */
    public static final double KeINTK_t_IntakeArmRaiseLowerDlyCmd = 0.025;


    /** KeINTK_t_AutoBallSeekTimeOut: Ball Intake
     * System Autonmous Ball Seek Time Out.  If in Autonomuous
     * Ball Pick Up a Ball is not detected the Ball Intake 
     * command will Time-Out after this time expires.
     */
    public static final double KeINTK_t_AutoBallSeekTmeOut = 4.00;

        /** KeINTK_t_AutoBallHoldEndDly: Ball Intake
     * System Autonomous Ball Hold Time out.  Once
     * the a Ball is detected by the First Intake Advance
     * Sensor a timer is started. If the Ball Intake Command
     * will Time-Out after this time, used if the ball does
     * not advance to the 2nd Intake Adance Sensor. 
     */
    public static final double KeINTK_t_AutoBallHoldEndTmeOut = 1.00;




  /**************************************************/
  /*  Ball Feed Motor Power Calibrations            */
 	/**************************************************/ 

   	/** KeINTK_r_TgtIntakePwrFeed: Ball Intake System Control 
     * Commmanded Intake Motor Normalized Power Command (0 to 1)
     * during Ball Feed. 
     */
    public static final double KeINTK_r_TgtIntakePwrFeed = 0.9;


	  /** KeINTK_r_TgtAdvancePwrFeed: Ball Intake System Control 
     * Commmanded Advance Motor Normalized Power Command (0 to 1)
     * during Ball Feed.
     */
    public static final double KeINTK_r_TgtAdvancePwrFeed = 0.3;


	  /** KeINTK_r_TgtIntakePwrShoot: Ball Intake System Control 
     * Commmanded Intake Motor Normalized Power Command (0 to 1)
     * during Ball Shoot.
     */
    public static final double KeINTK_r_TgtIntakePwrShoot = 0.3;


	  /** KeINTK_r_TgtAdvancePwrShoot: Ball Intake System Control 
     * Commmanded Intake Motor Normalized Power Command (0 to 1)
     * during Ball Shoot.
     */
    public static final double KeINTK_r_TgtAdvancePwrShoot = 0.8;



  /**************************************************/
  /*  Ball Feed Motor Speed Calibrations            */
 	/**************************************************/ 
   
   	/** KeINTK_n_TgtIntakeCmdFeed: Ball Intake System Control 
     * Commmanded Intake Motor Speed Command (Revs Per 100ms)
     * during Ball Feed. 
     */
    public static final double KeINTK_n_TgtIntakeCmdFeed = 6000;


   	/** KeINTK_n_TgtAdvanceCmdFeed: Ball Intake System Control 
     * Commmanded Advance Motor Speed Command (Revs Per 100ms)
     * during Ball Feed. 
     */
    public static final double KeINTK_n_TgtAdvanceCmdFeed = 2000;


   	/** KeINTK_n_TgtIntakeCmdShoot: Ball Intake System Control 
     * Commmanded Intake Motor Speed Command (Revs Per 100ms)
     * during Ball Shoot. 
     */
    public static final double KeINTK_n_TgtIntakeCmdShoot = 6000;


   	/** KeINTK_n_TgtAdvanceCmdShoot: Ball Intake System Control 
     * Commmanded Advance Motor Speed Command (Revs Per 100ms)
     * during Ball Shoot. 
     */
    public static final double KeINTK_n_TgtAdvanceCmdShoot = 6000;


    
  /****************************************************/
  /*  Ball Intake Control PID Coefficients            */
 	/****************************************************/	 	
    
	  /** KeINTK_K_Prop: Ball Intake System Proporational 
     * Control Gain. 
     */
    public static final double KeINTK_K_InProp = 0.1;


	  /** KeINTK_K_Intgl: Ball Intake System Integral 
     * Control Gain. 
     */
    public static final double KeINTK_K_InIntgl = 0;


	  /** KeINTK_K_Deriv: Ball Intake System Derivative
     * Control Gain. 
     */
    public static final double KeINTK_K_InDeriv = 0;


	  /** KeINTK_K_FdFwd: Ball Intake System Feed Fowrward
     * Control Gain. 
     */
    public static final double KeINTK_K_InFdFwd = 0.1;


	  /** KeINTK_r_IntglErrMaxEnbl: Ball Intake System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeINTK_r_InIntglErrMaxEnbl = 0.1;




  /****************************************************/
  /*  Ball Advance Control PID Coefficients           */
 	/****************************************************/	 	
    
	  /** KeINTK_K_Prop: Ball Advance System Proporational 
     * Control Gain. 
     */
    public static final double KeINTK_K_AdvProp = 0.1;


	  /** KeINTK_K_Intgl: Ball Advance Intake System
     * Integral Control Gain. 
     */
    public static final double KeINTK_K_AdvIntgl = 0;


	  /** KeINTK_K_Deriv: Ball Advance Intake System
     * Derivative Control Gain. 
     */
    public static final double KeINTK_K_AdvDeriv = 0;


	  /** KeINTK_K_FdFwd: Ball Advance Intake System Feed
     * Fowrward Control Gain. 
     */
    public static final double KeINTK_K_AdvFdFwd = 0.1;


	  /** KeINTK_r_IntglErrMaxEnbl: Ball Advance System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeINTK_r_AdvIntglErrMaxEnbl = 0.1;


}
