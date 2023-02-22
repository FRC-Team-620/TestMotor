package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

	// I'm giving you a basic structure and you guys job is to fill it out
	// Hopefully you will learn something from this process and that it will help
	// you with being able to do it on your own
	// Just giving you a helping hand for your first time round

	// Device id's are CAN pin numbers and you will be seeing a lot more of them in
	// the future so I suggest you get used to it
	// Second argument is a Enum and the long and short of it is it's words that
	// represent a number in a way that makes it more readable, but in this case
	// it's just idenifing that the motor we have plugged into that CAN slot is a
	// brushless motor
	private CANSparkMax leftFrontMotor = new CANSparkMax(1,
			MotorType.kBrushless);
	private CANSparkMax rightFrontMotor = new CANSparkMax(2,
			MotorType.kBrushless);
	private CANSparkMax leftRearMotor = new CANSparkMax(3,
			MotorType.kBrushless);
	private CANSparkMax rightRearMotor = new CANSparkMax(4,
			MotorType.kBrushless);


	// private double commandedXSpeed = 0.0;
	// private double commandedZRotation = 0.0;
	// private boolean commandedAllowTurnInPlace = false;


	// private boolean headingLock = false;



	private double commandedSpeed = 0.0;
	private double commandedCurvature = 0.0;
	private boolean commandedAllowTurnInPlace = false;
	private DifferentialDrive differentialDrive;

	/** Creates a new Drivetrain. */
	public Drivetrain() {
		SmartDashboard.putNumber("Drivetrain/leftFrontCANID", leftFrontMotor.getDeviceId());
		SmartDashboard.putNumber("Drivetrain/rightFrontCANID", rightFrontMotor.getDeviceId());
		SmartDashboard.putNumber("Drivetrain/leftRearCANID", leftRearMotor.getDeviceId());
		SmartDashboard.putNumber("Drivetrain/rightRearCANID", rightRearMotor.getDeviceId());

		setupMotors();
		setupFollowerMotors();
		initSensors();

	
		differentialDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
	}

	public void setBrake(boolean brake) {
		IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
		leftFrontMotor.setIdleMode(mode);
		rightFrontMotor.setIdleMode(mode);
		leftRearMotor.setIdleMode(mode);
		rightRearMotor.setIdleMode(mode);
	}

	private void setupMotors() {
		leftFrontMotor = setupMotor(leftFrontMotor);
		rightFrontMotor = setupMotor(rightFrontMotor);
		leftRearMotor = setupMotor(leftRearMotor);
		rightRearMotor = setupMotor(rightRearMotor);
	}

	private void initSensors() {


		// leftFrontEncoder = leftFrontMotor.getEncoder();
		// rightFrontEncoder = rightFrontMotor.getEncoder();

		// leftFrontEncoder.setPositionConversionFactor(DriveConstants.metersPerEncoderTick);
		// rightFrontEncoder.setPositionConversionFactor(DriveConstants.metersPerEncoderTick);

	}

	@Override
	public void periodic() {
		double rotationInput = this.commandedCurvature;
		differentialDrive.curvatureDrive(this.commandedSpeed, rotationInput, this.commandedAllowTurnInPlace);

	}

	

	private CANSparkMax setupMotor(CANSparkMax motor) {
		// You need to make the motor have the following settings that you can set
		// through the various motor methods:
		// Open loop ramp rate (time it takes to reach max acceleration in seconds) =
		// 0.2
		motor.setOpenLoopRampRate(0.2);
		// Smart current limit (limits on the current motors can draw even under full
		// load) = 60
		motor.setSmartCurrentLimit(60);
		// Idle mode (the mode that the motors are in when they are not being told to
		// move) = IdleMode.kBrake
		motor.setIdleMode(IdleMode.kBrake);

		return motor;
	}
	
	

	// TODO: Remove create reset odometry class This will cause bugs with the
	// odometry
	public void resetEncoders() {
	}

	private void setupFollowerMotors() {
		// You need to make the rear motors on both sides of the drivetrain follow their
		// respective front motors
		// This is where we will invert motors as needed when we get to testing the
		// drivetrain
		rightRearMotor.follow(rightFrontMotor);
		leftRearMotor.follow(leftFrontMotor);

		rightFrontMotor.setInverted(false);
		leftFrontMotor.setInverted(true);
	}

	// Sets the differential drive using the method curvatureDrive
	public void setCurvatureDrive(double speed, double rotationInput, boolean quickTurn) {
		// System.out.println("" + speed+' '+ rotationInput+' '+ quickTurn);
		SmartDashboard.putNumber("Drivetrain/speed", speed); // TODO: Remove update values in periodic
		SmartDashboard.putNumber("Drivetrain/rotationInput", rotationInput); // TODO: Remove update values in periodic
		this.commandedSpeed = speed;
		this.commandedCurvature = rotationInput;
		this.commandedAllowTurnInPlace = quickTurn;
	}

	public void setRightMotors(double speed) { // TODO: Do we need these methods?
		rightFrontMotor.set(speed);
	}
	public void setLeftMotors(double speed) {// TODO: Do we need these methods?
		leftFrontMotor.set(speed);
	}

	
}
