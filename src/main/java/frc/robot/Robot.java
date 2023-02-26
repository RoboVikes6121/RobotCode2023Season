// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.modules.CTREConfigs;
import frc.robot.subsystems.Arm;
//import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import pabeles.concurrency.ConcurrencyOps.Reset;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
   UsbCamera camera1;
   UsbCamera camera2;
  // UsbCamera camera3;
  // UsbCamera camera4; 
  Timer m_timer = new Timer();
  private Command m_autonomousCommand;
  // Joystick operator = new Joystick(1);
  public static CTREConfigs ctreConfigs = new CTREConfigs();
  private RobotContainer m_robotContainer;
  public Swerve swerve = new Swerve();
  public Arm arm = new Arm();
  public Intake intake = new Intake();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //FIXME: reset the cancoders when we turn robot on 
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    ctreConfigs = new CTREConfigs();
    Constants.initialpose = swerve.getPose();
    m_robotContainer = new RobotContainer();
    
     camera1 = CameraServer.startAutomaticCapture(0);
     camera2 = CameraServer.startAutomaticCapture(1);
    // camera3 = CameraServer.startAutomaticCapture(2);
    // camera4 = CameraServer.startAutomaticCapture(3); 

    try{
      arm.armInit();
      
    }catch(Exception e){
      System.out.println(e);
    }

    
  }

  /*
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    swerve.periodic();
    SmartDashboard.putNumber("arm encoder", arm.getEncoderValue()); 
    // SmartDashboard.putNumber("mvp", DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
    // SmartDashboard.putNumber("speed",SdsModuleConfigurations.MK4I_L2.getDriveReduction());
    // SmartDashboard.putNumber("mav", DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    
    //SmartDashboard.putNumber("joy", RobotContainer.m_controller.getLeftX());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //CANCoder.Reset;
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //swerve.resetOdometry(Constants.initialpose);
    // swerve.resetModulesToAbsolute();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putNumber("arm encoder", Arm.getEncoderValue()); //Moved to robotPeriodic

    //Good practice to avoid the posibility of setting a motor speed multiple times in a single iteration of code.
    
    if(m_robotContainer.m_operator.getRawButton(16)){
      // arm.armExtend();
      arm.armToPosition(1000); //When button is held move arm to 1000 encoder ticks
    }else if(m_robotContainer.m_operator.getRawButton(15)){
      // arm.armRetract();
      arm.armToPosition(75); //When button is held move arm to 75 encoder ticks
      //Typically want to avoid move the arm all the way back in, so that it doesn't hit any hard stops(metal)
    }else if(m_robotContainer.m_operator.getRawButton(14)){
      // arm.armStop();
      arm.armToPosition(2000);  //When button is held move arm to 2000 encoder ticks
    }else{
      arm.writeArm(m_robotContainer.m_operator.getRawAxis(1));
    }
    if(m_robotContainer.m_operator.getRawButton(7)){
      intake.Pickup();
    }else if(m_robotContainer.m_operator.getRawButton(8)){
      intake.Drop();
    }else{  //Need to have a default, if no buttons are held, then the motor stops.
      intake.intakestop();
    }
  }
  //If the motor controller(TalonSRX/TalonFX) is not set to a new value/speed for a certain amount of time, it will enter a safety mode and disable itself.
  //This is a safety function.

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}



/*
* to be deleted and made pretty
*/
// public void dAuton(){
// // get distance from out gyro
// // determine how far we want to travel
// // compare our gyro distance reading and determine if we still need to move forward
// // keep track of other coordinates, x, y, z
// if (m_timer.get() < 0){
//   driveforward(0);
// }
// else if(m_timer.get() > 0 & m_timer.get() < 1.5){
//   driveforward(.2);
// }
// }

// public static void driveforward(double power) {
// }

// public static int m_timer() {
//   return 0;
//  }
}
