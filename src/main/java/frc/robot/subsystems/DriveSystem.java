/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CounterBase;


public class DriveSystem extends SubsystemBase {
  private DifferentialDrive differentialDrive;
  private SpeedControllerGroup leftDrive;
  private SpeedControllerGroup rightDrive;

  private Encoder leftDriveEncoder;
  private Encoder rightDriveEncoder;

  private Solenoid shifter;

  /**
   * Creates a new DriveSystem.
   */
  public DriveSystem() {
    var leftFront = new WPI_VictorSPX(1);
		var leftBack = new WPI_VictorSPX(4);
		var leftTop = new WPI_VictorSPX(3);
		leftTop.setInverted(true);

  	var rightFront = new WPI_VictorSPX(0);
		var rightBack = new WPI_VictorSPX(5);
		var rightTop = new WPI_VictorSPX(2);
		rightTop.setInverted(true);

		leftDrive = new SpeedControllerGroup(leftFront, leftBack, leftTop);
		rightDrive = new SpeedControllerGroup(rightFront, rightBack, rightTop);

    leftDriveEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		leftDriveEncoder.setDistancePerPulse(Math.PI * 2 * (2.0 / 12) / 2048.0);

		rightDriveEncoder = new Encoder(4, 5, true, CounterBase.EncodingType.k4X);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		rightDriveEncoder.setDistancePerPulse(Math.PI * 2 * (2.0 / 12) / 2048.0);

		shifter = new Solenoid(0); 

    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveRobot(double move , double turn) {
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

	public boolean isLowGear() {
    return shifter.get();

	}//end getGear	

	public void setGear(boolean gear) {//false = high gear, true = low gear
		shifter.set(gear);
	}

	public void stopMotors() {
		differentialDrive.stopMotor(); 
	}

}
