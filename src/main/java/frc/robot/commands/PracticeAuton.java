// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;

/** Add your docs here. */
public class PracticeAuton {{
if(Robot.m_timer() > 0){
  Robot.driveforward(0);
}
    else if(Robot.m_timer() > 0 & Robot.m_timer() < 1.5 ){
        Robot.driveforward(.2);

        }
}}


