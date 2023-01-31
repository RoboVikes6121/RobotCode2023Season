// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class RunAuton {

    
    
    // public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath){
    //         return new SequentialCommandGroup(
    //             new InstantCommand (() -> {
    //                 if(isFirstPath){
    //                     this.resetOdometry(traj.getInitialHolonomicPose());
    //                 }
    //             }),
    //             new PPSwerveControllerCommand(
    //                 traj,
    //                 this :: getPose, 
    //                 this.kinematics,
    //                 new PIDController(0, 0, 0),
    //                 new PIDController(0, 0, 0),
    //                 new PIDController(0, 0, 0),
    //                 this :: setModuleStates,
    //                 true,
    //                 this
    //             )
    //         );
    // }
}
