// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static int flR = 43;
  public static int frR = 20;
  public static int blR = 11;
  public static int brR = 40;

  public static int flD = 5;
  public static int frD = 6;
  public static int blD = 3;
  public static int brD = 4;

  public static int flE = 2;
  public static int frE = 1;
  public static int blE = 3;
  public static int brE = 0;

  public static int rotation = 56;
  public static int shooter = 0;
  public static int feeder = 0;
  

  public static double flEOffset = -1.48;
  public static double frEOffset = 0.187;
  public static double blEOffset = -2.41;
  public static double brEOffset = 0.42;

  public static double driveKp = 1.0;
  public static double driveKi = 0.0;
  public static double driveKd = 0.0;

  public static double RPStoMPS = 4*Math.PI*2.54/100 ;

  public static int intakeSliderLimitSwitchID = 0;


  public static int intakeMotorID = 9;
  public static int turretMotorID = 10;
  public static int shooterMotorID = 33;
  public static int rollerMotorID = 56;
  public static int hoodMotorID = 36;
  public static int intakeSliderMotorID = 45;
  public static int feederMotorID = 35;


}