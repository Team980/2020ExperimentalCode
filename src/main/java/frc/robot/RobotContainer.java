/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.ControlPanelWheel;
import frc.robot.subsystems.DriveSystem;
//import frc.robot.subsystems.DriveSystem;
//import frc.robot.subsystems.DriveSystemPIDver2;
import frc.robot.subsystems.DriveSystemPIDver3;
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
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //private final DriveSystem driveSystem;
  //private final DriveSystemPIDver2 driveSystem;
  private final DriveSystemPIDver3 driveSystem3;

  private ControlPanelWheel cpWheel;
  //private Joystick throttle;
  //private Joystick wheel;
  private XboxController xBox;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //throttle = new Joystick(2);
    //wheel = new Joystick(1);
    xBox = new XboxController(0);

    // driveSystem = new DriveSystem();

    // driveSystem.setDefaultCommand(new RunCommand(() -> 
    //   driveSystem.driveRobot(applyDeadband(-xBox.getY(GenericHID.Hand.kLeft), Constants.MOVE_DEADBAND) , 
    //   applyDeadband(xBox.getX(GenericHID.Hand.kRight), Constants.TURN_DEADBAND)), driveSystem));

    driveSystem3 = new DriveSystemPIDver3();
    driveSystem3.setDefaultCommand(new RunCommand(() -> 
      driveSystem3.driveRobot(-xBox.getY(GenericHID.Hand.kLeft) , xBox.getX(GenericHID.Hand.kRight)), driveSystem3));

    // cpWheel = new ControlPanelWheel();
    // cpWheel.setDefaultCommand(new RunCommand(() -> {
    //   if (xBox.getTriggerAxis(Hand.kLeft) > 0){
    //     cpWheel.spinWheel(applyDeadband(-xBox.getTriggerAxis(Hand.kLeft), Constants.SPINNER_DEADBAND));
    //   }
    //   else{
    //     cpWheel.spinWheel(applyDeadband(xBox.getTriggerAxis(Hand.kRight), Constants.SPINNER_DEADBAND));
    //   }
    // } , cpWheel));
      


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
    //return m_autoCommand;
    return null;
  }


  	/**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
  public double applyDeadband(double value, double deadband) {
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
