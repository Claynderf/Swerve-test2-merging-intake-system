// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.BALL_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.INTAKE_LAUNCHER_MOTOR_2_ID;
import static frc.robot.Constants.FuelConstants.INTAKE_LAUNCHER_MOTOR_3_ID;
import static frc.robot.Constants.FuelConstants.INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.INTAKING_FEEDER_POWER;
import static frc.robot.Constants.FuelConstants.INTAKING_LAUNCHER_POWER;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LAUNCHING_FEEDER_POWER;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_POWER;
import static frc.robot.Constants.FuelConstants.SPIN_UP_FEEDER_POWER;

public class CANFuelSubsystem extends SubsystemBase {
 
  private final SparkMax intakeLauncherRoller;
  private final SparkMax intakeLauncherRoller2; 
  private final SparkMax intakeLauncherRoller3;
  private final SparkMax feederRoller;
  private final SparkMax ballMotor;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    intakeLauncherRoller2 = new SparkMax(INTAKE_LAUNCHER_MOTOR_2_ID, MotorType.kBrushless);
    intakeLauncherRoller3 = new SparkMax(INTAKE_LAUNCHER_MOTOR_3_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless); 
    ballMotor = new SparkMax(BALL_MOTOR_ID, MotorType.kBrushless);
    
    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true).idleMode(SparkBaseConfig.IdleMode.kBrake);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig intakeLauncherRoller2Config = new SparkMaxConfig();
    intakeLauncherRoller2Config.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller2Config.inverted(false).idleMode(SparkBaseConfig.IdleMode.kBrake);
    intakeLauncherRoller2.configure(intakeLauncherRoller2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intakeLauncherRoller3Config = new SparkMaxConfig();
    intakeLauncherRoller3Config.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller3Config.inverted(true).idleMode(SparkBaseConfig.IdleMode.kBrake);
    intakeLauncherRoller3.configure(intakeLauncherRoller3Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig ballMotorConfig = new SparkMaxConfig();
    ballMotorConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    ballMotorConfig.inverted(true).idleMode(SparkBaseConfig.IdleMode.kCoast);
    ballMotor.configure(ballMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_POWER);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_LAUNCHER_POWER);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_POWER);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_POWER);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_POWER);
    
  }

  public void setIntakeLauncherRoller(double power) {
    intakeLauncherRoller.set(power);
    intakeLauncherRoller2.set(power);
    intakeLauncherRoller3.set(power);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    feederRoller.set(power);
  }
  // A method to set the power of the intake roller
  public void setBallMotor(double power) {
    ballMotor.set(power);
  }

  // A method to stop the rollers
  public void stop() {
    
    intakeLauncherRoller.set(0);
    intakeLauncherRoller2.set(0);
    intakeLauncherRoller3.set(0);
    feederRoller.set(0);
    ballMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
