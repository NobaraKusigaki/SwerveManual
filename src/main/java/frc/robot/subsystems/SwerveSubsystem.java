// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
  Constants.MOTOR_LEFT_DRIVER_FRONT, 
  Constants.MOTOR_LEFT_ANGULAR_FRONT, 
  Constants.CANCODER_FRONT_LEFT);

  private final SwerveModule backLeft = new SwerveModule(
  Constants.MOTOR_LEFT_DRIVER_BACK, 
  Constants.MOTOR_LEFT_ANGULAR_BACK, 
  Constants.CANCODER_BACK_LEFT);

  private final SwerveModule frontRight = new SwerveModule(
  Constants.MOTOR_RIGHT_DRIVER_FRONT, 
  Constants.MOTOR_RIGHT_ANGULAR_FRONT, 
  Constants.CANCODER_FRONT_RIGHT);

  private final SwerveModule backRight = new SwerveModule(
  Constants.MOTOR_RIGHT_DRIVER_BACK, 
  Constants.MOTOR_RIGHT_ANGULAR_BACK, 
  Constants.CANCODER_BACK_RIGHT); 
 
  private final Pigeon2 gyro = new Pigeon2(Constants.PIGEON_ID);

public Object getRobotHeading;
  

  public SwerveSubsystem() {
    updateSmartDashBoard();
     configModules();
    
     new Thread(() -> {
      try {
          Thread.sleep(1000);
          zerogyro();
      } catch (Exception e) {
          e.printStackTrace();
      }
  }).start(); 
  }
  
  public void configModules(){

    frontLeft.setInvertedDriveMotor(false);
    frontLeft.setInvertedAngMotor(false);
    frontLeft.setBrakeDriveMotor(IdleMode.kBrake);
    frontLeft.setBrakeAngMotor(IdleMode.kBrake);

    backLeft.setInvertedDriveMotor(false);
    backLeft.setInvertedAngMotor(false);
    backLeft.setBrakeDriveMotor(IdleMode.kBrake);
    backLeft.setBrakeAngMotor(IdleMode.kBrake);

    frontRight.setInvertedDriveMotor(false);
    frontRight.setInvertedAngMotor(false);
    frontRight.setBrakeDriveMotor(IdleMode.kBrake);
    frontRight.setBrakeAngMotor(IdleMode.kBrake);

    backRight.setInvertedDriveMotor(false);
    backRight.setInvertedAngMotor(false);
    backRight.setBrakeDriveMotor(IdleMode.kBrake);
    backRight.setBrakeAngMotor(IdleMode.kBrake);

  }

  public void zerogyro(){
    gyro.reset();
    
  }

  public double getRobotHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
    }
  public void updateSmartDashBoard(){
  SmartDashboard.putNumber("Robot Heading",getRotation2d().getDegrees());
}    

  public void drive(double x1, double y1, double x2) {
    double r = Math.hypot(Constants.kTrackWidth, Constants.kWheelLenght);

    double vertical_Direction = x1 - x2 * (Constants.kWheelLenght / r); 
    double vertical2_Direction = x1 + x2 * (Constants.kWheelLenght / r); 
    double horizontal_Direction = y1 - x2 * (Constants.kTrackWidth / r); 
    double horizontal2_Direction = y1 + x2 * (Constants.kTrackWidth / r); 

    double LB_driver = Speed(Math.hypot(vertical_Direction, horizontal_Direction)); //Adição do speed para suavizar velocidade ainda nn testada 
    double RB_driver = Speed(Math.hypot(vertical_Direction, horizontal2_Direction));
    double LF_driver = Speed(Math.hypot(vertical2_Direction, horizontal_Direction));
    double RF_driver = Speed(Math.hypot(vertical2_Direction, horizontal2_Direction));

    double LB_ang = Math.atan2(vertical_Direction, horizontal_Direction);
    double RB_ang = Math.atan2(vertical_Direction, horizontal2_Direction);
    double LF_ang = Math.atan2(vertical2_Direction, horizontal_Direction);
    double RF_ang = Math.atan2(vertical2_Direction, horizontal2_Direction);

    frontLeft.direction(LF_driver, LF_ang);
    frontRight.direction(RF_driver, RF_ang);
    backLeft.direction(LB_driver, LB_ang);
    backRight.direction(RB_driver, RB_ang);
}
public double Speed(double input) {
  return Math.max(Math.min(input, Constants.MAX_SPEED), -Constants.MAX_SPEED);
}

public void stopModules(){
  frontLeft.stop();
  backLeft.stop();
  frontRight.stop();
  backRight.stop();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}