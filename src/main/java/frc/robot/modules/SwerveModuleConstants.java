// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;

/**
 * Custom constants for each swerve module.
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * @param driveMotorID CAN ID of the driving motor
     * @param angleMotorID CAN ID of the agnle motor
     * @param canCoderID CAN ID of the encoder
     * @param angleoffset2 Offset angle of something
     */

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID,
        Rotation2d angleoffset2) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleoffset2;
    }
}