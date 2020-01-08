/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveSystem driveSystem;
  private final TeleopDrive teleopDrive;

  private WPI_TalonSRX endEffectorIntakeMotor;

  private WPI_TalonSRX liftMotor;
	private Encoder liftEncoder;

  // wrist
  private WPI_TalonSRX wristMotor;
  //private Potentiometer wristPotentiometer;

  private Joystick throttle;
  private Joystick wheel;
  private XboxController xBox;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

		// end effector
		endEffectorIntakeMotor = new WPI_TalonSRX(13); 

		// lift
		liftMotor = new WPI_TalonSRX(15);
		liftEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);

		// wrist
		/*wristPotentiometer = new Potentiometer(0);
		wristMotor = new WPI_TalonSRX(11);
    wristMotor.setName("wrist contreller");*/
    
    throttle = new Joystick(0);
    wheel = new Joystick(1);
    xBox = new XboxController(2);

    driveSystem = new DriveSystem(wheel, throttle);
    teleopDrive = new TeleopDrive(driveSystem);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getTeleopDriveCommand(){
    //return teleopDrive;
    return new RunCommand(() -> driveSystem.driveRobot(applyDeadband(-throttle.getY(), Constants.MOVE_DEADBAND) , applyDeadband(wheel.getX(), Constants.TURN_DEADBAND)), driveSystem);
  }

	public double getLiftJoystickValue() {
		double value = -xBox.getY(Hand.kRight);
		return applyDeadband(value, Constants.LIFT_DEADBAND);
	}

	public double getWristJoystickValue() {
		double value = -xBox.getY(Hand.kLeft);
		return applyDeadband(value, Constants.WRIST_DEADBAND);
  }
  
  	/**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
  private static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } 
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    } 
    else {
      return 0.0;
    }
  } 
}
