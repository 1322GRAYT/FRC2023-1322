/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * SwrvMap: Identifies the location of the Swerve Drive Caddies, i.e. maps each
 * caddy to an integer value for array indexing.
 */
public class SwrvMap {
  public static final int FtRt = 0; // Swerve Drive Caddy: Front Right
  public static final int FtLt = 1; // Swerve Drive Caddy: Front Left
  public static final int RrLt = 2; // Swerve Drive Caddy: Rear  Left
  public static final int RrRt = 3; // Swerve Drive Caddy: Rear  Right
  public static final int NumOfCaddies = 4; //Total Number of Swerve Drive Caddies

  public static final int RtSd = 0; // Swerve Drive Bank: Right Side
  public static final int LtSd = 1; // Swerve Drive Bank: Left  Side
  public static final int NumOfBanks = 2; //Total Number of Swerve Drive Caddies

}
