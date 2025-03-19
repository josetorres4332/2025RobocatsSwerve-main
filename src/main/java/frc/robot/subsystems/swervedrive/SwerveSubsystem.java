// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");

  private final SwerveDrive swerveDrive; //Swerve drive object
  
  public SwerveSubsystem()
  {
    
    try
    {
      // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){

    return run(() -> {
      // Makes the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), 
                                                                      translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 
                                                                      Constants.Scalar),
                                                                      Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), 
                                                                      true, 
                                                                      false);});       
    }
}