// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);
  private Solenoid solenoid1 = new Solenoid(42, PneumaticsModuleType.CTREPCM, 0);
  private Solenoid solenoid2 = new Solenoid(42, PneumaticsModuleType.CTREPCM, 1);
  private Compressor compressor = new Compressor(42, PneumaticsModuleType.REVPH);



  /** Creates a new ExampleSubsystem. */
  public MotorSubsystem() {
    motor.setOpenLoopRampRate(0.2);
    motor.setSmartCurrentLimit(60);
    motor.setIdleMode(IdleMode.kBrake);

    compressor.enableDigital();
  }

  public void setSpeed(double speed){
    motor.set(speed);
  }
  public double getEncoderCount() {
    return motor.getEncoder().getPosition();
  }
  public void setSolenoid1(boolean state) {
    solenoid1.set(state);
  }
  public void setSolenoid2(boolean state) {
    solenoid2.set(state);
  }
  public boolean getStateSolenoid1() {
    return solenoid1.get();
  }
  public boolean getStateSolenoid2() {
    return solenoid2.get();
  }
  public double getPotVoltage() {
    //return motor.getAnalog(Mode.kAbsolute).getVoltage();
    return motor.getAnalog(Mode.kAbsolute).getVoltage();
  }



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //motor.set(0.3);
    System.out.println(motor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
