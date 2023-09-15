package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public CANSparkMax anchorMotor;
    public RelativeEncoder anchorEncoder;
    public SparkMaxPIDController anchorPIDController;

    public Arm(){
        this.anchorMotor = new CANSparkMax(Constants.Arm.kAnchorPort, CANSparkMax.MotorType.kBrushless);
        configureMotor();
        this.anchorEncoder = this.anchorMotor.getEncoder();
        configureEncoders();
        this.anchorPIDController = this.anchorMotor.getPIDController(); 
    }

    public void configureMotor(){
        this.anchorMotor.setInverted(Constants.Arm.kInverted);
        this.anchorMotor.setIdleMode(IdleMode.kBrake);
        this.anchorMotor.setSmartCurrentLimit(60); 
        this.anchorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kMinAngle); 
        this.anchorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kMaxAngle);
        this.anchorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        this.anchorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    public void configureEncoders() {
        this.anchorEncoder.setPositionConversionFactor(Constants.Arm.kRatio);
        this.anchorEncoder.setPosition(Constants.Arm.kContracted);  
     }

     public void configureControllers() {
        this.anchorPIDController.setP(Constants.Arm.kP);
        this.anchorPIDController.setI(Constants.Arm.kI);
        this.anchorPIDController.setD(Constants.Arm.kD);
        this.anchorPIDController.setFF(Constants.Arm.kFF);
    }

    public double getAnchorAngle(){
        return this.anchorEncoder.getPosition();
    }

    public double getAnchorVelocity(){
        return this.anchorEncoder.getVelocity();
    }

}