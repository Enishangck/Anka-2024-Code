package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleSubsystem extends SubsystemBase{
    private final CANSparkMax angleMaster;
    private final CANSparkMax angleSlave;
    public RelativeEncoder angleMasterEncoder;
    public RelativeEncoder angleSlaveEncoder;
    private final SparkPIDController angleMasterPID;
    private final SparkPIDController angleSlavePID;

    public AngleSubsystem() {
        angleMaster = new CANSparkMax(18, MotorType.kBrushless);
        angleSlave = new CANSparkMax(19, MotorType.kBrushless);
        angleMaster.restoreFactoryDefaults();
        angleSlave.restoreFactoryDefaults();
        angleMaster.setIdleMode(IdleMode.kBrake);
        angleSlave.setIdleMode(IdleMode.kBrake);
        angleSlave.follow(angleMaster);
        
        angleMasterPID = angleMaster.getPIDController();
        angleMasterEncoder = angleMaster.getEncoder();
        angleMasterEncoder.setPosition(0);
        angleMasterPID.setP(0.58186);
        angleMasterPID.setD(0.42105);
        angleMasterPID.setOutputRange(-0.8, 0.8);
        angleMasterPID.setSmartMotionMaxVelocity(10000000, 0);
        angleMasterPID.setSmartMotionMaxAccel(10000000, 0);
        angleMasterPID.setSmartMotionAllowedClosedLoopError(1, 0);
        
        angleSlavePID = angleSlave.getPIDController();
        angleSlaveEncoder = angleSlave.getEncoder();
        angleSlaveEncoder.setPosition(0);
        angleSlavePID.setP(0.58186);
        angleSlavePID.setD(0.42105);
        angleSlavePID.setOutputRange(-0.8, 0.8);
        angleSlavePID.setSmartMotionMaxVelocity(10000000, 0);
        angleSlavePID.setSmartMotionMaxAccel(10000000, 0);
        angleSlavePID.setSmartMotionAllowedClosedLoopError(1, 0);
    }

    public Command SetAngle(double pos){
        return runOnce(()-> {
            angleMasterPID.setReference(pos, CANSparkBase.ControlType.kPosition);
            angleSlavePID.setReference(pos, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command ManuelAngle(double speed){
        return runOnce(()-> {
            angleMaster.set(speed);
            angleSlave.set(speed);
        });
    }

    public Command Reset(){
        return runOnce(()-> {
            angleMasterEncoder.setPosition(0);
            angleSlaveEncoder.setPosition(0);
        });
    }
}
