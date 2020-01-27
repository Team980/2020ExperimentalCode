/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystemPIDver2 extends SubsystemBase {
  private double Kp = 1;
  private double Ki = 0;
  private double Kd = 0;
  private SimpleMotorFeedforward ff;

  private SpeedControllerGroup leftDrive;
  private SpeedControllerGroup rightDrive;

  private Encoder leftEncoder;
  private Encoder rightEncoder;

  private PIDController leftController;
  private PIDController rightController;

  private DifferentialDriveKinematics robotPIDDrive;

  /**
   * Creates a new DriveSystemPIDver2.
   */
  public DriveSystemPIDver2() {
    var leftFront = new WPI_TalonSRX(1);
		var leftBack = new WPI_TalonSRX(2);
		var leftTop = new WPI_TalonSRX(3);
    leftTop.setInverted(true);
    leftDrive = new SpeedControllerGroup(leftFront , leftBack , leftTop);
		leftEncoder = new Encoder(0, 1);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		leftEncoder.setDistancePerPulse(Math.PI * 2 * (Constants.WHEEL_RADIUS_METRIC) / 2048.0);

		var rightFront = new WPI_TalonSRX(4);
		var rightBack = new WPI_TalonSRX(5);
		var rightTop = new WPI_TalonSRX(6);
    rightTop.setInverted(true);
    rightDrive = new SpeedControllerGroup(rightFront , rightBack , rightTop);
		rightEncoder = new Encoder(2, 3);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
    rightEncoder.setDistancePerPulse(Math.PI * 2 * (Constants.WHEEL_RADIUS_METRIC) / 2048.0);
    
    leftController = new PIDController(Kp, Ki, Kd);
    rightController = new PIDController(Kp, Ki, Kd);
    ff = new SimpleMotorFeedforward( 1 , Constants.FEED_FOWARD_GAIN);

    robotPIDDrive = new DifferentialDriveKinematics(1); //Estimate one meter width


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = ff.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = ff.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftController.calculate(leftEncoder.getRate(),
        speeds.leftMetersPerSecond);
    final double rightOutput = rightController.calculate(rightEncoder.getRate(),
        speeds.rightMetersPerSecond);
    leftDrive.setVoltage(leftOutput + leftFeedforward);
    rightDrive.setVoltage(rightOutput + rightFeedforward);
  }

    /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = robotPIDDrive.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
}
