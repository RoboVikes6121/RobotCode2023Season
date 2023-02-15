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
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Swerve;
import frc.robot.modules.AutoBase;
/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class RunAuton extends AutoBase {
 //   Swerve swerve;

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public RunAuton(Swerve swerve) {
        super(swerve);
     //PathPlannerTrajectory p0 = PathPlanner.loadPath("test", 6, 3);
        PathPlannerTrajectory p0 = PathPlanner.loadPath("test", new PathConstraints(6,3));
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(p0);
        PathPlannerState initialState = p0.getInitialState();
       System.out.println("here is your sample trajectory");
        PathPlannerState examState = (PathPlannerState) p0.sample(1.65);
        System.out.println(examState.velocityMetersPerSecond);
        // TurnToAngle firstCommand = new TurnToAngle(swerve, 250, false);

        addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand);
// package frc.robot.autos;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.modules.AutoBase;
// import frc.robot.subsystems.Swerve;

//   /* 
//    * Autonomous that aligns limelight then executes a tracjectory.
//   */
//   public class P0 extends AutoBase {
//       Swerve swerve; 
    
//       public P0(Swerve swerve) {
//         super(swerve);
//         PathPlannerTrajectory p0 = PathPlanner.loadPath("P0", 6, 3);
//         PPSwerveControllerCommand firstCommand = baseSwerveCommand(p0);
//         PathPlannerState initialState = p0.getInitialState();
//         //TurnToAngle firstCommand = new TurnToAnlgle(swerve, 250, false)

//         addCommands(new InstantCommand(() -> swerve.zeroGyro()));
//         new InstantCommand(
//             () -> swereve.resentOdometry(new Pose2d(initialState.poseMeters.getTranslation())),
//                 initialState.holonomicRotation),
//                 firstCommand);
    
//     }

// }
    

//      /*
//       *  * Autonomous that aligns limelight then executes a tracjectory.
//   */
//       */
//   }
//     }
// ArrayList<PathPlannerTrajectory> pathGroup1 =(ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("test", new PathConstraints(4, 3)); 

// ArrayList<PathPlannerTrajectory> pathGroup2 =(ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("test", 
// new PathConstraints(4, 3),
// new PathConstraints(2, 2),
// new PathConstraints(3, 3)); 

//HashMap<String, Command> eventMap =new HashMap<>(); 

//eventMap.put("marker1", new PrintCommand("Passes marker 1"));

//eventMap.put("intakeDown", new IntakeDown()); 
// SwerveAutoBuilder autoBuilder =new SwerveAutoBuilder(
//     swerve::getPose,
//     swerve::resetPose,
//     swerve::Kinematics,
//     new PIDConstants(5.0, 0.0, 0.0),
//     new PIDConstants(0.5, 0.0, 0.0),
//     swerve::setModuleStates,
//     eventMap,
//     true,
//     swerve
// );

//Command test = autoBuilder.test(pathGroup1); 
}
}