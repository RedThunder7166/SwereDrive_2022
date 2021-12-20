// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {



  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_turnEncoder;
  // Driving encoder uses the integrated FX encoder
  // e.g. testMotor.getSelectedSensorPosition();

  // PID controller for velocity. DO NOT SET kD.  It is redundant as setVoltage() already controls this
  private final PIDController m_drivePIDController = 
    new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  
  //Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPidController = 
    new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  /** Creates a new SwerveModule. **/
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPorts,
      double angleZero) {
    
    // Initialize the motors
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);

    
    // Configure the encoders for both motors
    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    this.m_turnEncoder = new CANCoder(turningEncoderPorts);
    this.m_turnEncoder.configMagnetOffset(-angleZero);

  }

  //Returns the current state of the module

  public SwerveModuleState getState(){
    double m_speedMetersPerSecond = 
      ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =  
      ((2*Math.PI)/360) * m_turnEncoder.getPosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }

  public void setDesiredState(SwerveModuleState desiredState){

    double m_speedMetersPerSecond = 
      ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();
    
    SmartDashboard.putNumber("m_speedMetersPerSecond", m_speedMetersPerSecond);
    SmartDashboard.putNumber("Falcon SelectedSensorVelocity", m_driveMotor.getSelectedSensorVelocity());

    double m_turningRadians =  
      ((2*Math.PI)/360) * m_turnEncoder.getPosition();

    SmartDashboard.putNumber("m_turningRadians", m_turningRadians);
    SmartDashboard.putNumber("Turning Degrees", m_turnEncoder.getPosition());

    //Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningRadians));

    SmartDashboard.putNumber("desiredState", desiredState.speedMetersPerSecond);

    //Calculate the drive output from the drive PID controller
    final double driveOutput =
      m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond);

    final var turnOutput = 
      m_turningPidController.calculate(m_turningRadians, state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller
    m_driveMotor.setVoltage(driveOutput); 
    m_turningMotor.setVoltage(turnOutput);

    SmartDashboard.putNumber("PID driveOutput", driveOutput);
    SmartDashboard.putNumber("PID turnOutput", turnOutput);



  }

  public void resetEncoders() {
    m_turnEncoder.setPosition(0);
    m_driveMotor.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}