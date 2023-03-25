// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {

  CANSparkMax wristMotor = new CANSparkMax(8, MotorType.kBrushless);
  TalonSRX wristAbsoluteEncoder = new TalonSRX(9);

  double wristEncoderPosition;

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    this.wristMotor.setSmartCurrentLimit(20);
    this.wristMotor.setOpenLoopRampRate(0.2);
    this.wristMotor.setIdleMode(IdleMode.kCoast);

    this.wristAbsoluteEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    this.updateWristAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    this.updateWristAbsoluteEncoder();

    SmartDashboard.putNumber("MotorSubsystem/wristAbsoluteEncoder", this.getWristAbsoluteEncoderPosition());
  }

  public void setWristMotor(double speed) {
    this.wristMotor.set(speed);
  }

  public double getWristAbsoluteEncoderPosition() {
    return this.wristEncoderPosition;
  }

  private void updateWristAbsoluteEncoder() {
    this.wristEncoderPosition = this.wristAbsoluteEncoder.getSelectedSensorPosition();
  }

}