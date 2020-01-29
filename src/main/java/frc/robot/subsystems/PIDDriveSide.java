/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class PIDDriveSide implements SpeedController {
    private SpeedController motor;
    private Encoder encoder;

    private PIDController controller;

    private double setPoint;
    private boolean inverted;

    public PIDDriveSide(SpeedController motor, Encoder encoder) {
        this.motor = motor;
        this.encoder = encoder;
        this.setPoint = 0;
        this.inverted = false;

        controller = new PIDController(1.0, 0, 0);
    }

    public void set(double speed) {
        setPoint = speed * (inverted? -1 : 1);
        controller.setSetpoint(setPoint);
        
        double output = controller.calculate(encoder.getRate()/MAX_DRIVE_SPEED_FPS);
        SmartDashboard.putNumber("proportional correction", output);
        motor.set(output + FEED_FORWARD_MULTIPLIER*setPoint);
    }

    public double get() {
        return setPoint * (inverted? -1 : 1);
    }

    public void setInverted(boolean isInverted) {
        inverted = isInverted;
    }

    public boolean getInverted() {
        return inverted;
    }

    public void disable() {
        motor.disable();
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void pidWrite(double output) {
        set(output);
    }
}
