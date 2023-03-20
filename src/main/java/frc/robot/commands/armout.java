// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class armout extends CommandBase {
  /** Creates a new armout. */
  Arm arm;
  Timer m_timer = new Timer();
  public armout(Arm a) {
    arm = a; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("armout");
    m_timer.start();
    arm.armToPosition(84000);
   

    System.out.println("arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(arm.getEncoderValue() >= 83000 || m_timer.get() > 3){
    return(true);}
    return false;
  }
}
