// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm {
    static TalonFX arm = new TalonFX(8);
//SmartDashboard.putnumber.
TimeOfFlight proximitySensor = new TimeOfFlight(Constants.proxSensor); // the sensor we will use to check arm extension
    
public static void armInit(){
    arm.setInverted(false);
    arm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 1));
    arm.configClearPositionOnLimitF(false, 0);
    arm.configClearPositionOnLimitR(false, 0);
    arm.configClearPositionOnQuadIdx(false, 0);
    arm.setSelectedSensorPosition(0, 0, 0);

}
public static void armExtend(){
    arm.set(ControlMode.PercentOutput, -.3);
}
public static void armStop(){
    arm.set(ControlMode.PercentOutput, 0);
}
public static void armRetract(){
    arm.set(ControlMode.PercentOutput, .3);
}
public static void writeArm(double y){
    if(y<=.2 && y>=-.2){
        arm.set(ControlMode.PercentOutput, 0);

    }
else{
    arm.set(ControlMode.PercentOutput, y/2);
}
}
}
