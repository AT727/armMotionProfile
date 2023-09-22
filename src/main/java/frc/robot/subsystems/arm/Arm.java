package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        initTuneControllers();
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
        this.anchorEncoder.setVelocityConversionFactor(Constants.Arm.velocityConversationFactor);
        this.anchorEncoder.setPosition(Constants.Arm.kContracted);  
     }

     public void configureControllers() {
        this.anchorPIDController.setP(Constants.Arm.kP);
        this.anchorPIDController.setI(Constants.Arm.kI);
        this.anchorPIDController.setD(Constants.Arm.kD);
    }
    
    public void initTuneControllers(){
        SmartDashboard.putNumber("anchorKP", SmartDashboard.getNumber("anchorKP", Constants.Arm.kP));
        SmartDashboard.putNumber("anchorKI", SmartDashboard.getNumber("anchorKI", Constants.Arm.kI));
        SmartDashboard.putNumber("anchorKD", SmartDashboard.getNumber("anchorKD", Constants.Arm.kD));
    }

     public void tuneControllers() {
        double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
        double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
        double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);

        this.anchorPIDController.setP(anchorKP);
        this.anchorPIDController.setI(anchorKI);
        this.anchorPIDController.setD(anchorKD);
     }

    public double getAnchorAngle(){
        return this.anchorEncoder.getPosition();
    }

    public double getAnchorVelocity(){
        return this.anchorEncoder.getVelocity();
    }

    public TrapezoidProfile.State getAnchorState(){
        return new TrapezoidProfile.State(getAnchorAngle(), getAnchorVelocity());
    }

    public void setPosition(double angle, double arbFFVoltage){
        this.anchorPIDController.setReference(angle, CANSparkMax.ControlType.kPosition, 2, arbFFVoltage, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }


    @Override
    public void periodic() {

        tuneControllers();

   }
}
