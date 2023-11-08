// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

/** Add your docs here. */
public class Intake{
   public TalonSRX Motor = new TalonSRX(9);

public void Pickup(){
    Motor.set(ControlMode.PercentOutput, Constants.PickupSpeed);
}
public void Drop(){
    Motor.set(ControlMode.PercentOutput, Constants.OutTakeSpeed);
}
public void intakestop(){
    Motor.set(ControlMode.PercentOutput, 0);
}

public void intakeSpeed(double speed){
    Motor.set(ControlMode.PercentOutput, speed);
}

}
