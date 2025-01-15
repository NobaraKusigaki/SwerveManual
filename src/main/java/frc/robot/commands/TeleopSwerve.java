// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TeleopSwerve extends Command {
  private Joystick joy;
  private SwerveSubsystem subsystem;
  private double x1_joy, y1_joy, x2_joy;
  private int pov;

  public TeleopSwerve(SwerveSubsystem subsystem, Joystick joy) {
  this.joy = joy;
  this.subsystem = subsystem ; 
  addRequirements(subsystem);
  }

 
  @Override
  public void initialize() {}

@Override
public void execute() {
  this.x1_joy = Math.abs(joy.getRawAxis(Constants.LEFT_STICK_X)) > Constants.kDeadband ? joy.getRawAxis(Constants.LEFT_STICK_X) : 0;
  this.y1_joy = Math.abs(-joy.getRawAxis(Constants.LEFT_STICK_Y)) > Constants.kDeadband ? -joy.getRawAxis(Constants.LEFT_STICK_Y) : 0;
  this.x2_joy = Math.abs(joy.getRawAxis(Constants.RIGHT_ROT_AXIS)) > Constants.kDeadband ? joy.getRawAxis(Constants.RIGHT_ROT_AXIS) : 0;

  subsystem.drive(x1_joy,y1_joy,x2_joy);
}


  
  @Override
  public void end(boolean interrupted) {
    subsystem.stopModules();
  }

 
  @Override
  public boolean isFinished() {
    return false;
  }
}