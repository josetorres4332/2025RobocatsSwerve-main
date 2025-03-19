// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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

public class WristSubsystem extends SubsystemBase {

  private SparkMax wristMotor;
  private RelativeEncoder wristEncoder;
  private SparkClosedLoopController wristController;
  private double currentWristPos;

  private double currentVelocity = 0;

  public WristSubsystem() {

    wristMotor = new SparkMax(Constants.coralElbowID, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();
    wristController = wristMotor.getClosedLoopController();

    currentWristPos = wristEncoder.getPosition();

    SparkMaxConfig wristConfigMotor = new SparkMaxConfig();

    wristConfigMotor.softLimit
                                            .forwardSoftLimit(50)
                                            .forwardSoftLimitEnabled(true)
                                            .reverseSoftLimit(0)
                                            .reverseSoftLimitEnabled(true);

    wristConfigMotor
                                            .closedLoop
                                            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                            .p(0.1)
                                            .i(0)
                                            .d(0)
                                            .outputRange(-0.15, 0.15)
                                            .p(0.0001, ClosedLoopSlot.kSlot1)
                                            .i(0, ClosedLoopSlot.kSlot1)
                                            .d(0, ClosedLoopSlot.kSlot1)
                                            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                                            .outputRange(-0.15, 0.15, ClosedLoopSlot.kSlot1);
    
    wristConfigMotor.apply(wristConfigMotor).inverted(false);

    wristMotor.configure(wristConfigMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder.setPosition(0);

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

    public void goToDropLevel() {
        double wristPos = 15;
        wristController.setReference(wristPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        currentWristPos = wristPos;
    }

    public void goToWDefault() {
        double wristPos = 3;
        wristController.setReference(wristPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        currentWristPos = wristPos;
    }

    public void moveWrist(boolean up) {
        if (up) {
            setVelocity(.35, .05, wristMotor, true);
        } else {
            setVelocity(-.35, .05, wristMotor, false);
        }
        
        currentWristPos = wristEncoder.getPosition();
        System.out.println("wrist pos: " + currentWristPos);
    }

    public Command goToDropCommand() {
        return run(() -> goToDropLevel());
    }

    public Command goToWDefaultCommand() {
        return run(() -> goToWDefault());
    }

    public Command moveArmUpCommand() {
        return run(() -> moveWrist(true));
    }

    public Command moveArmDownCommand() {
        return run(() -> moveWrist(false));
    }

    public Command holdArmPositionCommand() {
        return run(() -> {
            System.out.println("holding arm pos: " + currentWristPos);
            wristController.setReference(currentWristPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }
}
