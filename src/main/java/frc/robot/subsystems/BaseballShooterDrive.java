/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BaseballShooterDrive extends SubsystemBase {
  Spark leftDrive;
  Spark rightDrive;
  DifferentialDrive robotDrive;
  /**
   * Creates a new BaseballShooterDrive.
   */
  public BaseballShooterDrive() {
    leftDrive = new Spark(0);
    rightDrive = new Spark(1);
    robotDrive = new DifferentialDrive(leftDrive, rightDrive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double throttle , double turn){
    robotDrive.arcadeDrive(throttle, turn);
  }
}
