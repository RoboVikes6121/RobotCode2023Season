// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class RunAuton extends AutoBase {
    Swerve swerve;

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public RunAuton(Swerve swerve) {
        super(swerve);
        PathPlannerTrajectory p0 = PathPlanner.loadPath("test", 6, 3);
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(p0);
        PathPlannerState initialState = p0.getInitialState();
        // TurnToAngle firstCommand = new TurnToAngle(swerve, 250, false);

        addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand);

    }
}