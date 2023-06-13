package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SingleMotorSubsystem;

public class SingleMotorCommand extends CommandBase{
    private SingleMotorSubsystem singleMotorSubsystem; 
    private XboxController control;
    public SingleMotorCommand(SingleMotorSubsystem singleMotorSubsystem, XboxController control){
        this.singleMotorSubsystem = singleMotorSubsystem;
        this.control = control;

        addRequirements(singleMotorSubsystem);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }


}
