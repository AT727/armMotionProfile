package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;


public final class Constants {
    public static final class Arm{
    public static final int kAnchorPort = 22;
    public static final boolean kInverted = true;
    public static final double kRatio = (90.0 - 13.0) / (27.0); // sigh, do this right later

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    public static double kErrorThreshold = 5.0;
    public static double kMaxDownwardOutput = -0.4;
    public static double kMaxUpwardOutput = 0.5;

    public static final double kContracted = 16.0;
    public static final double kMinAngle = 16.0;
    public static final double kMaxAngle = 95.0;

    public static final double ks = 0;
    public static final double kg = 0;
    public static final double kv = 0;
    public static final double ka = 0;
    public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);
    
    }
}