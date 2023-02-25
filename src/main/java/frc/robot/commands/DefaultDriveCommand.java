package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Subsystem[] m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(Subsystem[] drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        // m_drivetrainSubsystem.drive(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(
        //                 m_translationXSupplier.getAsDouble(),
        //                 m_translationYSupplier.getAsDouble(),
        //                 m_rotationSupplier.getAsDouble(),
        //                 m_drivetrainSubsystem.getGyroscopeRotation()
                        
        //         )
        // );
        SmartDashboard.putNumber("translation",  m_translationXSupplier.getAsDouble());
        SmartDashboard.putNumber("translation2", m_translationYSupplier.getAsDouble());
        SmartDashboard.putNumber("rotation", m_rotationSupplier.getAsDouble());
        //SmartDashboard.putNumber("drive",  m_drivetrainSubsystem.getGyroscopeRotation());
    }

    @Override
    public void end(boolean interrupted) {
        // m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
