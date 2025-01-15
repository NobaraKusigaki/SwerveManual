// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
private final SparkMax driveMotor;
private final SparkMax angMotor;
private final CANcoder cancoder;
private final PIDController anglePID;
private double maxVolts = 12;
public double lastSpd;
public double lastPose;
   public SwerveModule(int driveMotorID, int angleMotorID, int cancoderID) {
        driveMotor = new SparkMax(driveMotorID, SparkMax.MotorType.kBrushless);
        angMotor = new SparkMax(angleMotorID, SparkMax.MotorType.kBrushless);
        cancoder = new CANcoder(cancoderID);
        anglePID = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);

        anglePID.enableContinuousInput(-Math.PI, Math.PI);
            
   
    resetCANcoder();
  }

 
public void setInvertedDriveMotor(boolean inverted){
SparkMaxConfig config = new SparkMaxConfig();
config.inverted(inverted);
driveMotor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

public void setInvertedAngMotor(boolean inverted){
SparkMaxConfig config = new SparkMaxConfig();
config.inverted(inverted);
angMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

public void setBrakeDriveMotor(IdleMode brake){
SparkMaxConfig config = new SparkMaxConfig();
config.idleMode(brake);
driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void setBrakeAngMotor(IdleMode brake){
  SparkMaxConfig config = new SparkMaxConfig();
  config.idleMode(brake);
  angMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public double CANcoderValueInDegrees() {
  double rot = cancoder.getAbsolutePosition().getValueAsDouble();
  return Units.rotationsToDegrees(rot);
}
public void resetCANcoder(){
  cancoder.setPosition(0.0);
}

public void stop() {
  driveMotor.set(0);
  angMotor.set(0);
}

public void direction(double speed, double angle){
double absolutePosition = CANcoderValueInDegrees();
driveMotor.set(speed);
angMotor.set(anglePID.calculate(absolutePosition, angle));}

public void syncCANcoderToAngle() {
  
  
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}