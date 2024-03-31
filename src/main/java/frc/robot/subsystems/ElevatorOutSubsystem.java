package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorOutSubsystem extends SubsystemBase{
    CANSparkMax elevatorOUT = new CANSparkMax(24, MotorType.kBrushless);

    public ElevatorOutSubsystem(){
    elevatorOUT.restoreFactoryDefaults();
    elevatorOUT.setIdleMode(IdleMode.kBrake);
    elevatorOUT.setInverted(true);
    }

    public Command elevatorUp(){
     return runOnce(()-> {
            elevatorOUT.set(0.45);
        });
    }

    public Command elevatorDown(){
     return runOnce(()-> {
            elevatorOUT.set(-0.45);
        });
    }

    public Command stopElevatorOut(){
     return runOnce(()-> {
            elevatorOUT.set(0);
        });
    }
}
