 // Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.TeleopSwerve;
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

    public static final double stickDeadband = .1;
    public static final int proxSensor = 0;
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

    public static final class Swerve{
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23);   //TODO FIXME SWERVE BASE SIZE
        public static final double wheelBase = Units.inchesToMeters(23);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

<<<<<<< HEAD
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1
=======
        public static final double openLoopRamp = 1;
        public static final double closedLoopRamp = 1;
>>>>>>> parent of dcc1d51 (fixed driving)

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

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKS = (0.667 / 12);
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4; // meters per second
        public static final double maxAngularVelocity = 5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */

        /**
         * Front Left Module - Module 0.
         */
        public static final class Mod0 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13;
<<<<<<< HEAD
            public static final double angleOffset = 307.8;
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(310.67);
>>>>>>> parent of dcc1d51 (fixed driving)
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
<<<<<<< HEAD
            public static final double angleOffset = 152.6;
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(148.94);
>>>>>>> parent of dcc1d51 (fixed driving)
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
<<<<<<< HEAD
            public static final double angleOffset = 242.75;
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(241.51);
>>>>>>> parent of dcc1d51 (fixed driving)
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Right Module - Module 3.
         */
        public static final class Mod3 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
<<<<<<< HEAD
            public static final int canCoderID = 10;
            public static final double angleOffset = 58.3;
=======
            public static final int canCoderID = 10; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(56.42); 
>>>>>>> parent of dcc1d51 (fixed driving)
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

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    } // end auto constants
        public static Pose2d initialpose;

} // ends class constants
