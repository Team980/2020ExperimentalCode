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
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;
/**
 * Add your docs here.
 */
public class SpeedControllerPIDWrapper implements SpeedController {
    private PIDController pidController;
    private SimpleMotorFeedforward feedFowardController;

    private SpeedController speedController;
    private Encoder encoder;

    private double setSpeed;
    
    public SpeedControllerPIDWrapper(SpeedController speedController, Encoder encoder) {
        this.feedFowardController = new SimpleMotorFeedforward(0, PID_DRIVE_F);
        this.pidController = new PIDController(PID_DRIVE_P, PID_DRIVE_I, PID_DRIVE_D);

        this.speedController = speedController;
        this.encoder = encoder;
        this.setSpeed = 0;
    }

    public void set(double setSpeed) {
        this.setSpeed = setSpeed;
        double pidOutput = pidController.calculate(encoder.getRate()/MAX_DRIVE_SPEED_FPS, setSpeed);
        speedController.set(pidOutput + feedFowardController.calculate(setSpeed));

        SmartDashboard.putNumber("p correction", pidOutput);
        SmartDashboard.putNumber("f correction", feedFowardController.calculate(setSpeed));

    }

    public double get() {
        return setSpeed;
    }

    public void setInverted(boolean isInverted) {
        speedController.setInverted(isInverted);
    }

    public boolean getInverted() {
        return speedController.getInverted();
    }

    public void disable() {
        speedController.disable();
    }

    public void stopMotor() {
        set(0);
        speedController.stopMotor();
    }

    public void pidWrite(double setSpeed) {
        set(setSpeed);
    }
}
