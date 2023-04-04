 // Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.FileAlreadyExistsException;

//import org.apache.commons.exec.ShutdownHookProcessDestroyer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.TeleopSwerve;
import frc.robot.modules.COTSFalconSwerveConstants;
import frc.robot.modules.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final double OutTakeSpeed = -.7;
    public static final double PickupSpeed = .7;
    public static final double stickDeadband = .1;
    public static  Pose2d initialpose;
    //public static double maxSpeedMultiplier = 1;
    //public static final kDriveTick2Feet = 
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    // public static final double DRIVETRAIN_TRACKWIDTH_METERS = .6; // FIXME Measure and set trackwidth
    // /**
    //  * The front-to-back distance between the drivetrain wheels.
    //  *
    //  * Should be measured from center to center.
    //  */
    // public static final double DRIVETRAIN_WHEELBASE_METERS = .6; // FIXME Measure and set wheelbase
    
    // //public static final int DRIVETRAIN_PIGEON_ID = 22; //  FIXME Set Pigeon ID
    // public static final double MAX_SPEED_MOD = .26;
    

    // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
    // public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
    // public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; // FIXME Set front left steer encoder ID
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(307.8); // FIXME Measure and set front left steer offset

    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; // FIXME Set front right drive motor ID
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set front right steer motor ID
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set front right steer encoder ID
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(152.6); // FIXME Measure and set front right steer offset

    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back left drive motor ID
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set back left steer motor ID
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set back left steer encoder ID
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(242.75); // FIXME Measure and set back left steer offset

    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set back right drive motor ID
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; // FIXME Set back right steer motor ID
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10; // FIXME Set back right steer encoder ID
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(58.3); // FIXME Measure and set back right steer offset
    public static final int proxSensor = 0;
    public static final class Swerve{
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
        //FIXME prox sensor is not set to a known value yet
        
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.875);   //TODO FIXME SWERVE BASE SIZE
        public static final double wheelBase = Units.inchesToMeters(30.125);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = true;

        public static final double openLoopRamp = 1;
        public static final double closedLoopRamp = 1;

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;



        /* Drive Motor Characterization Values */
        // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKS = (0.667 / 12);
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 3; // meter per second went from 4 to 1
        
        public static final double maxAngularVelocity = 4;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Coast;

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

                    /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;
        
        /* Drive Motor PID Values */
        public static final double driveKP = .4; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;
        public static final double iLimit = .5;


        /* Module Specific Constants */

        /**
         * Front Left Module - Module 0.
         */
        public static final class Mod0 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(51.76);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /**
         * Front Right Module - Module 1.
         */

        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(210.14);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Left Module - Module 2.
         */
        public static final class Mod2 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.44);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Right Module - Module 3.
         */
        // FIXME canCoderID
        public static final class Mod3 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 10; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(301.22); 
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }
        /**
     * Autonomous constants for swerve bot.
     */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double kPXController = 0;
        public static final double kPYController = 3;
        public static final double kPThetaController = 0;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    } // end auto constants

    public static final class ArmConstants {
        public static final double kArmP = 0.5;
        public static final double kArmI = 0.2;
        public static final double kArmD = 0.2;
        public static final double kArmIZone = 100;
        public static final double kArmPeakOutput = 0.5;
        public static final double kArmAllowedError = 50;
    }
        //SmartDashboard.putNumber("kArmP", kArmP);
        //SmartDashboard.putNumber(kArmP,"");
} // ends class constants
