/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double MOVE_DEADBAND = 0.1;
	public static double TURN_DEADBAND = 0.1;
	public static double LIFT_DEADBAND = 0.1;
	public static double WRIST_DEADBAND = 0.1;

	public static final double MAX_DRIVE_SPEED_FPS = 16.0;
	public static double MAX_DRIVE_SPEED_MPS = MAX_DRIVE_SPEED_FPS * .3048;
	public static double MAX_ANGULAR_SPEED = 2 * Math.PI; // one rotation per second

	public static double WHEEL_RADIUS = 2.0 / 12; //in feet
	public static double WHEEL_RADIUS_METRIC  = 2 * .0254;// in meters
	public static double FEED_FOWARD_GAIN = 1 / MAX_DRIVE_SPEED_MPS;

}
