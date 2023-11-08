// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
//public class RunAuton extends AutoBase{

    //Swerve swerve;

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
//      */
//     public RunAuton(Swerve swerve) {
//         super(swerve);
//         PathPlannerTrajectory p0 = PathPlanner.loadPath("TurnLeft", 6, 3);
//         PathPlannerTrajectory p0 = PathPlanner.loadPath("backToStraight", 6, 3);
//         PPSwerveControllerCommand firstCommand = baseSwerveCommand(p0);
//         PathPlannerState initialState = p0.getInitialState();
//         // TurnToAngle firstCommand = new TurnToAngle(swerve, 250, false);

//         addCommands(new InstantCommand(() -> swerve.zeroGyro()),
//             new InstantCommand(
//                 () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
//                     initialState.holonomicRotation))),
//             firstCommand);

//      //private void configurationCommands(){
//         //AUTO_EVENT_MAP.put(key:"event1". new PrintCommand(message:"passes marker 1"));
//         //AUTO_EVENT_MAP.put(key:"event2". new PrintCommand(message:"passes marker 2"));
//         //AUTO_EVENT_MAP.put(key:"event3". new PrintCommand(message:"passes marker 3"));
//            // }
//     }
//     // build auto path commands
//     // ArrayList<PathPlannerTrajectory> autoPaths =
//     //     PathPlanner.loadPathGroup(
//     //         "TurnLeft",
//     //         AUTO_MAX_SPEED_METERS_PER_SECOND,
//     //         AUTO_EVENT_ACCELERATION_METERS_PER_SECOND_SQUARED);
//     // private ArrayList<PathPlannerTrajectory> auto1Paths;
//     //     Command autoTest = 
//     //         new SequentialCommandGroup(
//     //             new FollowPathWithEvents(
//     //                 new FollowPath(autoPaths.get(0), Swerve, true),
//     //                 auto1Paths.get(0).getMarkers(),
//     //                 AUTO_EVENT_MAP),
//     //             new InstantCommand(Swerve::enableXstace, Swerve),
//     //             new WaitCommand(0),
//     //             new InstantCommand(Swerve::disableXstance, Swerve),
//     //             new FollowPathWithEvents(
//     //                 new FollowPath(auto1Paths.get(1), Swerve, false),
//     //                 auto1Paths.get(1).getMarkers(),
//     //                 AUTO_EVENT_MAP));

//     //                 autoChooser.addDefaultOption["Do Nothing", new InstantCommand())}; 

//     //                 autoChooser.addOption[]
//     // public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath){
//     //         return new SequentialCommandGroup(
//     //             new InstantCommand (() -> {
//     //                 if(isFirstPath){
//     //                     this.resetOdometry(traj.getInitialHolonomicPose());
//     //                 }
//     //             }),
//     //             new PPSwerveControllerCommand(
//     //                 traj,
//     //                 this :: getPose, 
//     //                 this.kinematics,
//     //                 new PIDController(0, 0, 0),
//     //                 new PIDController(0, 0, 0),
//     //                 new PIDController(0, 0, 0),
//     //                 this :: setModuleStates,
//     //                 true,
//     //                 this
//     //             )
//     //         );
//     // }
// }
