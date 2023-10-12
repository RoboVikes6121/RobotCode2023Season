// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class StabilizerController {
    private boolean autoBalanceXMode;
    private boolean autoBalanceYMode;
    private double kOffBalanceThresholdDegrees = 10;
    private double kOnBalanceThresholdDegrees = 5;
    private Swerve m_Swerve = new Swerve();


    public double stabX(){
        double pitchAngleDegrees  = m_Swerve.getPitch();

        if ( !autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode && (Math.abs(pitchAngleDegrees) <=Math.abs(kOnBalanceThresholdDegrees))) {
            autoBalanceXMode = false;
        }

        if (autoBalanceXMode) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            return Math.sin(pitchAngleRadians) * -1;
        }else{
            return(0);
        }
    }
    public double stabY(){
        double rollAngleDegrees  = m_Swerve.getRoll();

        if ( !autoBalanceYMode && (Math.abs(rollAngleDegrees) >= Math.abs(kOffBalanceThresholdDegrees))) {
            autoBalanceYMode = true;
        }
        else if ( autoBalanceYMode && (Math.abs(rollAngleDegrees) <=Math.abs(kOnBalanceThresholdDegrees))) {
            autoBalanceYMode = false;
        }

        if (autoBalanceYMode) {
            double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
            return Math.sin(rollAngleRadians) * -1;
        }else{
            return(0);
        }
    }
}
