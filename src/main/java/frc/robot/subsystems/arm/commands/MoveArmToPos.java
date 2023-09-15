package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;



public class MoveArmToPos extends CommandBase {
    Arm arm;
    double setpoint;
    TrapezoidProfile trapezoidProfile;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 1);
    

    public MoveArmToPos(Arm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;

        //create new profile, input current position & velocity, set goal to stop at new position
        this.trapezoidProfile = new TrapezoidProfile(this.constraints, new TrapezoidProfile.State(setpoint, 0), arm.getAnchorState());
        addRequirements(arm);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //calculate position & velocity every loop
        TrapezoidProfile.State setpointState = trapezoidProfile.calculate(0.02);
        //calculate feedforward for goal position & velocity
        double arbFFVoltage = Constants.Arm.ARM_FEEDFORWARD.calculate(setpointState.position, setpointState.velocity);
        //set position & feedforward
        arm.setPosition(setpointState.position, arbFFVoltage);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
}