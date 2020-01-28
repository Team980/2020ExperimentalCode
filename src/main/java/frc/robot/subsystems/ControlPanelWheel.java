/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanelWheel extends SubsystemBase {
  private CANSparkMax wheelSpinner;
  private CANEncoder wSEncoder;
  private double FPS;
  /**
   * Creates a new ControlPanelWheel.
   */
  public ControlPanelWheel() {
    wheelSpinner = new CANSparkMax(11 , MotorType.kBrushless);
    wSEncoder = wheelSpinner.getEncoder();
    FPS = 0.0;

  }

  @Override
  public void periodic() {
    FPS = wSEncoder.getVelocity() * Math.PI * .5 * 60;
    SmartDashboard.putNumber("Control Panel Driver Speed", FPS);
    SmartDashboard.putNumber("Control Panel Wheel Speed", FPS * 3.0 / 16);
    // This method will be called once per scheduler run
  }

  public void spinWheel(double speed){
    wheelSpinner.set(speed);
  }
}
