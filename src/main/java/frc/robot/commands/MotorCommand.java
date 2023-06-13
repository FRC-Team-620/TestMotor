// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class MotorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private MotorSubsystem motorSubsystem;
  private CommandXboxController controller;
  //private PIDController pidLoop = new PIDController(1, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MotorCommand(MotorSubsystem motorSubsystem, CommandXboxController controller) {
    this.motorSubsystem = motorSubsystem;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pidLoop.setSetpoint(1);
    //motorSubsystem.setSpeed(MathUtil.clamp(pidLoop.calculate(motorSubsystem.getEncoderCount()), -1, 1));
    //System.out.println("WORKKKKKK");
    //motorSubsystem.setSpeed(0.4);
    // if (controller.a().getAsBoolean()) {
    //   motorSubsystem.setSolenoid1(!motorSubsystem.getStateSolenoid1());
    // }
    // if (controller.b().getAsBoolean()) {
    //   motorSubsystem.setSolenoid2(!motorSubsystem.getStateSolenoid2());
    // }
    // if(controller.x().getAsBoolean()) {
    //   motorSubsystem.setSpeed(-0.4);
    // }
    // else {
    //   double netSpeed = 0;//;
    //   double max = Math.max(controller.getRightTriggerAxis(),controller.getLeftTriggerAxis());
    //   double min = Math.min(controller.getRightTriggerAxis(),controller.getLeftTriggerAxis());
    //   netSpeed = max-min;
    //   if(max == controller.getLeftTriggerAxis()){
    //     netSpeed = -netSpeed;
    //   }
    //   motorSubsystem.setSpeed(netSpeed);
    // }
    // System.out.println(motorSubsystem.getEncoderCount());
    SmartDashboard.putNumber("PotPosition", motorSubsystem.getAbsEncoderPosition());
    double exp = -114.440596296 * (motorSubsystem.getAbsEncoderPosition() - 2.49) - 63;
    SmartDashboard.putNumber("Expected Angle", exp);
    motorSubsystem.setSpeed2(controller.getLeftY());
    //motorSubsystem.setSpeed(controller.getRightY());
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}