/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDSpeedControllerGroup;

public class DriveSystemPIDver3 extends SubsystemBase {
  private double Kp = 1;
  private double Ki = 0;
  private double Kd = 0;
  private SimpleMotorFeedforward ff;

  private PIDSpeedControllerGroup leftDrive;
  private PIDSpeedControllerGroup rightDrive;

  private Encoder leftEncoder;
  private Encoder rightEncoder;

  private PIDController leftController;
  private PIDController rightController;

  private DifferentialDrive robotPIDDrive;


  
  /**
   * Creates a new DriveSystemPIDver3.
   */
  public DriveSystemPIDver3() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
