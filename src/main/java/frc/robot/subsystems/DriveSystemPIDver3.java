/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystemPIDver3 extends SubsystemBase {
  private PIDSpeedControllerGroup leftDrive;
  private PIDSpeedControllerGroup rightDrive;

  private Encoder leftEncoder;
  private Encoder rightEncoder;

  private DifferentialDrive robotPIDDrive;


  
  /**
   * Creates a new DriveSystemPIDver3.
   */
  public DriveSystemPIDver3() {
    var leftFront = new WPI_TalonSRX(1);
		var leftBack = new WPI_TalonSRX(2);
		var leftTop = new WPI_TalonSRX(3);
    leftTop.setInverted(true);
		leftEncoder = new Encoder(0, 1);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		leftEncoder.setDistancePerPulse(Math.PI * 2 * (Constants.WHEEL_RADIUS) / 2048.0);
    leftDrive = new PIDSpeedControllerGroup(leftEncoder , leftFront , leftBack , leftTop);

		var rightFront = new WPI_TalonSRX(4);
		var rightBack = new WPI_TalonSRX(5);
		var rightTop = new WPI_TalonSRX(6);
    rightTop.setInverted(true);
		rightEncoder = new Encoder(2, 3);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
    rightEncoder.setDistancePerPulse(Math.PI * 2 * (Constants.WHEEL_RADIUS) / 2048.0);
    rightDrive = new PIDSpeedControllerGroup(rightEncoder , rightFront , rightBack , rightTop);
    leftDrive.enable();
    rightDrive.enable();
    robotPIDDrive = new DifferentialDrive(leftDrive, rightDrive);
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive Velocity", leftEncoder.getRate());
    SmartDashboard.putNumber("Right Drive Velocity", rightEncoder.getRate());
    SmartDashboard.putNumber("Left Drive Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Drive Distance", rightEncoder.getDistance());
    // This method will be called once per scheduler run
  }

  public void driveRobot(double move, double turn) {
		robotPIDDrive.arcadeDrive(move, turn);
	}

	public double getLeftDistance() {
		return leftEncoder.getDistance();
	}

	public double getRightDistance() {
		return rightEncoder.getDistance();
	}

	public void resetEncoderDistance() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	public double getLeftSpeed() {
		return leftEncoder.getRate();
	}

	public double getRightSpeed() {
		return rightEncoder.getRate();
	}

	public void stopMotors() {
		robotPIDDrive.stopMotor(); 
	}

}
