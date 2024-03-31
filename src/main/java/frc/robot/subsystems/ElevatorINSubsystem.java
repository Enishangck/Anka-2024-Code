package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorINSubsystem extends SubsystemBase{
    public final CANSparkMax elevatorIN;
    public final RelativeEncoder elevatorEncoder;;
    public final SparkPIDController elevatorPID;

    public ElevatorINSubsystem() {
        elevatorIN = new CANSparkMax(20, MotorType.kBrushless);
        elevatorIN.restoreFactoryDefaults();
        elevatorIN.setIdleMode(IdleMode.kBrake);
        
        elevatorPID = elevatorIN.getPIDController();
        elevatorEncoder = elevatorIN.getEncoder();
        elevatorEncoder.setPosition(0);
        elevatorPID.setP(0.58186);
        elevatorPID.setD(0.42105);
        elevatorIN.enableVoltageCompensation(8);
        elevatorPID.setOutputRange(-0.6, 0.6);
        elevatorPID.setSmartMotionMaxVelocity(10000000, 0);
        elevatorPID.setSmartMotionMaxAccel(10000000, 0);
        elevatorPID.setSmartMotionAllowedClosedLoopError(1, 0);
    }

    public Command SetElevator(double pos){
        return runOnce(()-> {
            elevatorPID.setReference(pos, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command ManuelElevator(double speed){
        return runOnce(()-> {
            elevatorIN.set(speed);
        });
    }

    public Command Reset(){
        return runOnce(()-> {
            elevatorEncoder.setPosition(0);
        });
    }
}
