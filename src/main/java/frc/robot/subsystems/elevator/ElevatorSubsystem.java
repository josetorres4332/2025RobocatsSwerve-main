// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax leftElevatorMotor;
  private SparkMax rightElevatorMotor;
  private RelativeEncoder leftEncoder;
  private SparkClosedLoopController elevatorController;
  private double currentElevatorPos;
  private double currentVelocity = 0;

  public ElevatorSubsystem() {

    

    leftElevatorMotor = new SparkMax(Constants.elevatorLeftMotorID, MotorType.kBrushless);
    leftEncoder = leftElevatorMotor.getEncoder();
    elevatorController = leftElevatorMotor.getClosedLoopController();

    currentElevatorPos = leftEncoder.getPosition();

    rightElevatorMotor = new SparkMax(Constants.elevatorRightMotorID, MotorType.kBrushless);

    SparkMaxConfig globalConfigMotor = new SparkMaxConfig();
    SparkMaxConfig rightConfigMotor = new SparkMaxConfig();

    globalConfigMotor.softLimit
                            .forwardSoftLimit(180)
                            .forwardSoftLimitEnabled(true)
                            .reverseSoftLimit(0)
                            .reverseSoftLimitEnabled(true);

     globalConfigMotor
                            .idleMode(IdleMode.kBrake)
                            .closedLoop
                            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            .p(0.1)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1)
                            .p(0.0001, ClosedLoopSlot.kSlot1)
                            .i(0, ClosedLoopSlot.kSlot1)
                            .d(0, ClosedLoopSlot.kSlot1)
                            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    rightConfigMotor.apply(globalConfigMotor).follow(Constants.elevatorLeftMotorID, true);

    leftElevatorMotor.configure(globalConfigMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevatorMotor.configure(rightConfigMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
   
    leftEncoder.setPosition(0);

  }

  public void setVelocity(double targetVelocity, double rampRate, SparkMax motor, Boolean up) {
        new Thread(() -> {
            while (Math.abs(targetVelocity - currentVelocity) > 0.1) { // Small threshold to stop ramping
                if(up){
                    if (targetVelocity > currentVelocity) {
                        currentVelocity += rampRate;// Change in speed per cycle
                    } else {
                        currentVelocity -= rampRate;
                    }
                }
                else{
                    if (targetVelocity < currentVelocity) {
                        currentVelocity -= rampRate;// Change in speed per cycle
                    } else {
                        currentVelocity += rampRate;
                    }

                }

                motor.set(currentVelocity); // currentVelocity/ Max RPM
                
                try {
                    Thread.sleep(50); // Small delay for smooth ramping
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            System.out.println("current velocity" + currentVelocity);
            
            motor.set(targetVelocity); // Final adjustment
        }).start();
  }

    public void goToLevel1() {
            double elevatorPos = 10;
            elevatorController.setReference(elevatorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
            currentElevatorPos = elevatorPos;
    }

    public void goToLevel2() {
        double elevatorPos = 40;
        elevatorController.setReference(elevatorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        currentElevatorPos = elevatorPos;
    }

    public void goToLevel3() {
        double elevatorPos = 80;
        elevatorController.setReference(elevatorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        currentElevatorPos = elevatorPos;
    }

    public void goToLevel4() {
        double elevatorPos = 118;
        elevatorController.setReference(elevatorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        currentElevatorPos = elevatorPos; 
    }

    public void goToDefault() {
        double elevatorPos = 5;
        elevatorController.setReference(elevatorPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        currentElevatorPos = elevatorPos;
    }

    public void moveElevator(boolean up) {
        if (up) {
            setVelocity(.45, .05, leftElevatorMotor, true);
        } else {
            setVelocity(-.45, .05, leftElevatorMotor, false);
        }       
        
        currentElevatorPos = leftEncoder.getPosition();
        System.out.println("moved elevator pos: " + currentElevatorPos);
    }

    public Command goToLevel1Command() {
        return run(() -> goToLevel1());
    }

    public Command goToLevel2Command() {
        return run(() -> goToLevel2());
    }

    public Command goToLevel3Command() {
        return run(() -> goToLevel3());
    }

    public Command goToLevel4Command() {
        return run(() -> goToLevel4());
    }

    public Command goToDefaultCommand() {
        return run(() -> goToDefault());
    }

    public Command moveElevatorUpCommand() {
        return run(() -> moveElevator(true));
    }

    public Command moveElevatorDownCommand() {
        return run(() -> moveElevator(false));
    }

    public Command holdElevatorPositionCommand() {
        return run(() -> {

            elevatorController.setReference(currentElevatorPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            System.out.println("holding elevator pos: " + currentElevatorPos);
        });
    }
    
}
