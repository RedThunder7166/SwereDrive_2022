// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final CANCoder m_turnEncoder;
  // Driving encoder uses the integrated FX encoder
  // e.g. testMotor.getSelectedSensorPosition();

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
      int turningEncoderPorts) {
    
    // Initialize the motors
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    
    // Configure the encoders for both motors
    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 0);
    this.m_turnEncoder = new CANCoder(turningEncoderPorts);
  }

  //Returns the current state of the module

  public SwerveModuleState getState(){
    double m_speedMetersPerSecond = 
      ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =  
      ModuleConstants.kTurningPositiontoRadians * m_turnEncoder.getPosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }

  public void setDesiredState(SwerveModuleState desiredState){

    double m_speedMetersPerSecond = 
      ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =  
      ModuleConstants.kTurningPositiontoRadians * m_turnEncoder.getPosition();

    //Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningRadians));

    //Calculate the drive output from the drive PID controller
    final double driveOutput =
      m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond);

    final var turnOutput = 
      m_turningPidController.calculate(m_turningRadians, state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
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