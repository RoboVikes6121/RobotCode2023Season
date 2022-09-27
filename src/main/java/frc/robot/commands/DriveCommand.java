// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Utilities;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveCommand extends CommandBase{
  public DriveCommand() {
    requires(DrivetrainSubsystem.getInstance());
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double forward = -Robot.getOI().getPrimaryJoystick().getRawaxis(1);
    forward = Utilities.deadband(forward);
    //square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = -Robot.getOI().getPrimaryJoystick().getRawaxis(1);
    strafe = Utilities.deadband(strafe);
    //square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

    double rotation = -Robot.getOI().getPrimaryJoystick().getRawaxis(1);
    rotation = Utilities.deadband(rotation);
    //square the forward stick
    rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

    DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true );
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
 

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
}
