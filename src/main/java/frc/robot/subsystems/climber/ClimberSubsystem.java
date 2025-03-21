// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private static final TalonSRX leftClimbMotor = new TalonSRX(Constants.leftclimberMotorID);
  private static final TalonSRX rightClimberMotor = new TalonSRX(Constants.rightClimberMotorID);

    public ClimberSubsystem(){


    }

    public Command climberStop() {
        return run(() -> motorStop());
    }

    public Command climberGoingDown() {
      return run(() -> motorForward());
  }

  public Command climberGoingUp() {
    return run(() -> motorReverse());
}

    public void motorStop(){
        leftClimbMotor.set(ControlMode.PercentOutput, 0);
        rightClimberMotor.set(ControlMode.PercentOutput, 0);
    }
    
    public void motorForward(){
        leftClimbMotor.set(ControlMode.PercentOutput, -0.5);
        rightClimberMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void motorReverse(){
      leftClimbMotor.set(ControlMode.PercentOutput, 0.5);
      rightClimberMotor.set(ControlMode.PercentOutput, -0.5);
  }
}
