// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
public class RunAuton {
    public command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath){
            return new SequentialCommandGroup(
                new InstantCommand (() -> {
                    if(isFirstPath){
                        this.resetOdometry(traj.getInitialHolonomicRose());
                    }
                }),
                new PPSwerveControllerCommand(
                    traj,
                    this : : getPose, 
                    this.kinematics,
                    new PIDController(0, 0, 0),
                    new PIDController(0, 0, 0),
                    new PIDController(0, 0, 0),
                    this : : setModuleStates,
                    true,
                    this
                )
            );
    }
}
