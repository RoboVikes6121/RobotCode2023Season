// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.event.TableModelEvent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import pabeles.concurrency.ConcurrencyOps.Reset;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class DriveCommand extends SubsystemBase {
private static final double TRACKWIDTH = 19.5;
private static final double WHEELBASE = 23.5;
private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0);
private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0);
private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0);
private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0);
private static DriveCommand instace;

private final SwerveModuleState frontleftModule = new Mk2SwerveModuleBuilder(
  new TalonFX( TRACKWIDTH / 2.0 , WHEELBASE / 2.0))
  .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
  .angleMotor(new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
  .driveMotor(new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
  .build();

  private final SwerveModule frontrightModule = new Mk2SwerveModuleBuilder(
  new Vector2( TRACKWIDTH / 2.0 , -WHEELBASE / 2.0))
  .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
  .angleMotor(new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
  .driveMotor(new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
  .build();

  private final SwerveModule backleftModule = new Mk2SwerveModuleBuilder(
    new Vector2( -TRACKWIDTH / 2.0 , WHEELBASE / 2.0))
    .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
    .angleMotor(new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
    .driveMotor(new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
    .build();

    private final SwerveModule backrightModule = new Mk2SwerveModuleBuilder(
  new Vector2( -TRACKWIDTH / 2.0 , -WHEELBASE / 2.0))
  .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
  .angleMotor(new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
  .driveMotor(new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), Mk2SwerveModuleBuilder.MotorType.TalonFX)
  .build();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  new Translation2d(TRACKWIDTH / 2.0 , WHEELBASE / 2.0),
  new Translation2d(TRACKWIDTH / 2.0 , -WHEELBASE / 2.0),
  new Translation2d(-TRACKWIDTH / 2.0 , WHEELBASE / 2.0),
  new Translation2d(-TRACKWIDTH / 2.0 , -WHEELBASE / 2.0)
 );
  private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

  public DriveCommand() {
    gyroscope.calibrate();
    gyroscope.setInverted(true); // You might not need to invert the gyro

    frontLeftModule.setName("Front Left");
    frontRightModule.setName("Front Right");
    backLeftModule.setName("Back Left");
    backRightModule.setName("Back Right");
}

  public static DriveCommand getInstance() {
    if (instance == null) {
      instance = new DriveCommand();
    }
    return instance;
  }
  public void periodic() {
    frontleftModule.updateSensors();
    frontrightModule.updateSensors();
    backleftModule.updateSensors();
    backrightModule.updateSensors();

    SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontleftModule.getCurrentAngle()));
    SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontrightModule.getCurrentAngle()));
    SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backleftModule.getCurrentAngle()));
    SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backrightModule.getCurrentAngle()));

    SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

    frontleftModule.updateState(TimedRobot.kDefaultPeriod);
    frontrightModule.updateState(TimedRobot.kDefaultPeriod);
    backleftModule.updateState(TimedRobot.kDefaultPeriod);
    backrightModule.updateState(TimedRobot.kDefaultPeriod);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
    rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
    ChassisSpeeds speeds;
    if (fieldOriented){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
    } else {
      speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    frontleftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
    frontrightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
    backleftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
    backrightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
  }

  public void resetGyroscope() {
    gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
