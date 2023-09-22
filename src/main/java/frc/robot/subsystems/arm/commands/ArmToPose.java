package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;



public class ArmToPose extends CommandBase {
    Arm arm;
    double setpoint;
    TrapezoidProfile trapezoidProfile;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        Constants.Arm.maxVel, 
        Constants.Arm.maxAccel);
    Timer timer = new Timer();
    double trapezoidPofileStartTime;
    

    public ArmToPose(Arm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        

        //create new profile, input current position & velocity, set goal to stop at new position
        this.trapezoidProfile = new TrapezoidProfile(this.constraints, new TrapezoidProfile.State(setpoint, 0), arm.getAnchorState());
        trapezoidPofileStartTime = timer.getFPGATimestamp();

        addRequirements(arm);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double currentTime = timer.getFPGATimestamp();
        //calculate position & velocity every loop
        TrapezoidProfile.State setpointState = trapezoidProfile.calculate(currentTime - trapezoidPofileStartTime);
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
