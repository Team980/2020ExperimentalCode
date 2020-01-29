/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonBasedCodeDrive extends SubsystemBase {
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  private WPI_TalonSRX leftBack = new WPI_TalonSRX(2);
  private WPI_TalonSRX leftTop = new WPI_TalonSRX(3);

  private WPI_TalonSRX rightFront = new WPI_TalonSRX(4);
  private WPI_TalonSRX rightBack = new WPI_TalonSRX(5);
  private WPI_TalonSRX rightTop = new WPI_TalonSRX(6);


  /**
   * Creates a new TalonBasedCodeDrive.
   */
  public TalonBasedCodeDrive() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
