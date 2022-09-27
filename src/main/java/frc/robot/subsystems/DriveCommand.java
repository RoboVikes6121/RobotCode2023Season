// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.event.TableModelEvent;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/** Add your docs here. */
public class DriveCommand extends Subsystem {
private static final double TRACKWIDTH = 19.5;
private static final double WHEELBASE = 23.5;
private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0);
private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0);
private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0);
private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0);
private static DrivetrainSubsystem instace;

private final SwerveModule frontleftModule = new Mk2SwerveModuleBuilder(
  new Vector2( TRACKWIDTH / 2.0 , WHEELBASE / 2.0))
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
  new Translation2d(-TRACKWIDTH / 2.0 , -WHEELBASE / 2.0),
 );
  private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

  public DrivetrainSubsystem(){
    gyroscope.calibrate();
    gyroscope.setInverted(true); //migh not need to invert gyro

    frontleftModule.setName("front left");
    frontrightModule.setName("front right");
    backleftModule.setName("back left");
    backrightModule.setname("back right");
  }

  public static DrivetrainSubsystem getInstance() {
    if (instance == null) {
      instance = new DrivetrainSubsystem();
    }
    return instance;
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
