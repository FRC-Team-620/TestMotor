package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SingleMotorSubsystem extends SubsystemBase{
    private CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
    public SingleMotorSubsystem(){
        motor.setSmartCurrentLimit(40);
        motor.setOpenLoopRampRate(3);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }
}