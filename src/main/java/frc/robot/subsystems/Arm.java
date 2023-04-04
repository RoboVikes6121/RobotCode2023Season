// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm {
    TalonFX arm = new TalonFX(8);
    TalonFXConfiguration armConfiguration = new TalonFXConfiguration();
   
//TimeOfFlight proximitySensor = new TimeOfFlight(Constants.proxSensor); // the sensor we will use to check arm extension
    
public void armInit(){
        //PID Initiliziation
    armConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    armConfiguration.peakOutputForward = 0.3;
    armConfiguration.peakOutputReverse = -0.3;
    
    armConfiguration.slot0.kP = Constants.ArmConstants.kArmP;
    armConfiguration.slot0.kI = Constants.ArmConstants.kArmI;
    armConfiguration.slot0.kD = Constants.ArmConstants.kArmD;
    armConfiguration.slot0.integralZone = Constants.ArmConstants.kArmIZone;
    armConfiguration.slot0.closedLoopPeakOutput = Constants.ArmConstants.kArmPeakOutput;
    armConfiguration.slot0.allowableClosedloopError = Constants.ArmConstants.kArmAllowedError;

    arm.configAllSettings(armConfiguration);

    arm.setInverted(false);
    arm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 1));
    arm.configClearPositionOnLimitF(false, 0);
    arm.configClearPositionOnLimitR(false, 0);
    arm.configClearPositionOnQuadIdx(false, 0);
    arm.setSelectedSensorPosition(0, 0, 0);

    arm.setNeutralMode(NeutralMode.Brake);

    arm.getSensorCollection().setIntegratedSensorPosition(0, 30);

}
public double getEncoderValue(){
    return arm.getSelectedSensorPosition();
}
public void armExtend(){
    arm.set(ControlMode.PercentOutput, -.3);
   // SmartDashboard.putNumber("Encoder", arm.getActiveTrajectoryPositio );

}
public void armStop(){
    arm.set(ControlMode.PercentOutput, 0);
}
public void armRetract(){
    arm.set(ControlMode.PercentOutput, -.3);
}
public void writeArm(double y){
    if(y<=.2 && y>=-.2){
        arm.set(ControlMode.PercentOutput, 0);

    }
else{
    arm.set(ControlMode.PercentOutput, y/3);
}
} // end write arm
//while loops will cause everything else on the robot to stop functioning. Loops = bad in this usage case
public void autoExtend(){
if(arm.getSelectedSensorPosition() < 1000){ 
    arm.set(ControlMode.PercentOutput, -.8);
}
arm.set(ControlMode.PercentOutput, 0);
}
public void autoRetract(){ 
    if(arm.getSelectedSensorPosition() > 100){ 
        arm.set(ControlMode.PercentOutput, .8);
    }
    arm.set(ControlMode.PercentOutput, 0);
}
public void armToPosition(int positionRequest){
    arm.set(TalonFXControlMode.Position, positionRequest, DemandType.ArbitraryFeedForward, 0);
}

}
