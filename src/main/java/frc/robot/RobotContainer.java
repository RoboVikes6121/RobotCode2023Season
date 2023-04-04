




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StabilizerController;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.Auton12;
import frc.robot.autos.Auton13;
import frc.robot.autos.Auton11;
import frc.robot.autos.Auton21;
import frc.robot.autos.Auton22;
import frc.robot.commands.TeleopSwerve;
//import frc.robot.subsystems.StabilizerController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  //private final StabilizerController m_StabilizerController = new StabilizerController();
  private final Swerve m_Swerve = new Swerve();
  private final Intake m_Intake = new Intake();
  private final Arm m_Arm = new Arm();
  private final StabilizerController m_StabilizerController = new StabilizerController();
  //public final XboxController m_controller = new XboxController(1);
  public final Joystick m_operator = new Joystick(0);
  public final Joystick m_Joystick = new Joystick(1); 
  /* Drive Controls */
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;

  /* Driver Buttons */
  //private final JoystickButton zeroGyro = new JoystickButton(m_Joystick, Joystick.ButtonType.kTrigger.value);
  private final JoystickButton robotCentric = new JoystickButton(m_Joystick, Joystick.ButtonType.kTop.value);

    // private final Joystick m_joystick = new Joystick(0);
    // private final Joystick m_joystick2 = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_Swerve.setDefaultCommand(new TeleopSwerve(m_Swerve, m_controller, Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));
    m_Swerve.setDefaultCommand(
      new TeleopSwerve(
          m_Swerve, 
          () -> m_Joystick.getRawAxis(translationAxis), 
          () -> m_Joystick.getRawAxis(strafeAxis), 
          () -> -m_Joystick.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
      )
  );}
   // while(m_Joystick.getRawButton(16)){
    /*  m_Swerve.setDefaultCommand(new TeleopSwerve(
      m_Swerve,
      () -> -modifyAxis(m_StabilizerController.stabX() * .2),
      () -> -modifyAxis(m_StabilizerController.stabY() * .2),
      () -> -modifyAxis(m_controller.getRightX() * (.2 * Math.PI)),
      () -> robotCentric.getAsBoolean()
      )
    );
    }*/

 // SmartDashboard.putNumber("lx", m_controller.getLeftX() );
    // Configure the button bindings
    //configureButtonBindings();
  //}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
  //   new Button(m_controller::getBackButton)
  //           // No requirements because we don't need to interrupt anything
  //           .whenPressed(m_Swerve::reset);

        /* Driver Buttons */
       // zeroGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new InstantCommand();
    return new Auton11(m_Swerve, m_Arm, m_Intake );
   // return new Auton12(m_Swerve, m_Arm, m_Intake);
     // return new Auton13(m_Swerve, m_Arm, m_Intake);
   // return new Auton21(m_Swerve, m_Arm, m_Intake);
    //return new Auton22(m_Swerve, m_Arm, m_Intake);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
    
  }
}
