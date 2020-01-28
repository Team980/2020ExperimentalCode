/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CounterBase;


public class DriveSystem extends SubsystemBase {
	private DifferentialDrive differentialDrive;
	private SpeedController leftDrive;
	private SpeedController rightDrive;

	private Encoder leftDriveEncoder;
	private Encoder rightDriveEncoder;
	
	//private Solenoid shifter;

	/**
	 * Creates a new DriveSystem.
	 */
  	public DriveSystem() {
		var leftFront = new WPI_TalonSRX(1);
		var leftBack = new WPI_TalonSRX(2);
		var leftTop = new WPI_TalonSRX(3);
		leftTop.setInverted(true);
		leftDriveEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		leftDriveEncoder.setDistancePerPulse(Math.PI * 2 * (2.0 / 12) / 2048.0);
		//leftDriveEncoder.setName("left drive encoder");

		var rightFront = new WPI_TalonSRX(4);
		var rightBack = new WPI_TalonSRX(5);
		var rightTop = new WPI_TalonSRX(6);
		rightTop.setInverted(true);
		rightDriveEncoder = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		rightDriveEncoder.setDistancePerPulse(Math.PI * 2 * (2.0 / 12) / 2048.0);
		//rightDriveEncoder.setName("right drive encoder");

		leftDrive = new PIDDriveSide(new SpeedControllerGroup(leftFront, leftBack, leftTop), leftDriveEncoder);
		rightDrive = new PIDDriveSide(new SpeedControllerGroup(rightFront, rightBack, rightTop), rightDriveEncoder);
		// leftDrive = new SpeedControllerGroup(leftFront, leftBack, leftTop);
		// rightDrive = new SpeedControllerGroup(rightFront, rightBack, rightTop);

		differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
  	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Encoder Velocity", leftDriveEncoder.getRate());
		SmartDashboard.putNumber("Right Encoder Velocity", rightDriveEncoder.getRate());
		SmartDashboard.putNumber("Left Encoder Distance", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder Distance", rightDriveEncoder.getDistance());
		// This method will be called once per scheduler run
	}

  	public void driveRobot(double move, double turn) {
		differentialDrive.arcadeDrive(move, turn);
	}

	public double getLeftDistance() {
		return leftDriveEncoder.getDistance();
	}

	public double getRightDistance() {
		return rightDriveEncoder.getDistance();
	}

	public void resetEncoderDistance() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
	}

	public double getLeftSpeed() {
		return leftDriveEncoder.getRate();
	}

	public double getRightSpeed() {
		return rightDriveEncoder.getRate();
	}

	public void stopMotors() {
		differentialDrive.stopMotor(); 
	}

}
