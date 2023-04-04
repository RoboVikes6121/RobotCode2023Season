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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Feedin;
import frc.robot.commands.FollowPath;
import frc.robot.commands.armIn;
import frc.robot.commands.armout;
import frc.robot.commands.feedout;
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class Auton13 extends AutoBase {
  Arm arm; 
  Intake intake;
 // HashMap<String, Command> eventMap = new HashMap<String, Command>();
    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public Auton13(Swerve swerve, Arm a, Intake i) {
        super(swerve);
        arm = a;
        intake = i;
        armout out = new armout(arm);
        feedout feedout = new feedout(intake);
        armIn in = new armIn(arm);
        Feedin feedin = new Feedin(intake);
        
        //eventMap.put("arm out", new SequentialCommandGroup(out, feedout, new ParallelCommandGroup(in)));
        // taking path off path planner 
        PathPlannerTrajectory p0 = PathPlanner.loadPath("13", new PathConstraints(1.5,3));
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(p0);
        PathPlannerState initialState = p0.getInitialState();
       System.out.println("here is your sample trajectory");
        PathPlannerState examState = (PathPlannerState) p0.sample(2.39);
        System.out.println(examState.velocityMetersPerSecond);
        


       addCommands(new SequentialCommandGroup(out,feedout,in));

            
            addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand);
            

 }
 
}