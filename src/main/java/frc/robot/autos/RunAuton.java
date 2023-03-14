// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Feedin;
import frc.robot.commands.FollowPath;
import frc.robot.commands.armIn;
import frc.robot.commands.armout;
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class RunAuton extends AutoBase {
  Arm arm; 
  Intake intake;
    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public RunAuton(Swerve swerve, Arm a) {
        super(swerve);
        arm = a;
        // taking path off path planner 
        PathPlannerTrajectory p0 = PathPlanner.loadPath("TurnLeft", new PathConstraints(4,3));
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(p0);
        PathPlannerState initialState = p0.getInitialState();
       System.out.println("here is your sample trajectory");
        PathPlannerState examState = (PathPlannerState) p0.sample(2.39);
        System.out.println(examState.velocityMetersPerSecond);
        armout out = new armout(arm);
        // TurnToAngle firstCommand = new TurnToAngle(swerve, 250, false);
      
       // moving arm out 
        // SequentialCommandGroup part1 =
        // new SequentialCommandGroup(firstCommand,(new StartEndCommand(() -> {
        //     arm.armToPosition(80000);
        // }, () -> {
        //     arm.armStop();
        // })).andThen(new Feedin(intake)));
       
       // reseting odometry 
        addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand, out, new Feedin(intake),  new armIn(arm));

 }
}