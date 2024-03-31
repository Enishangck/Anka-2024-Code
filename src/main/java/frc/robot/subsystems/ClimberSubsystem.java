package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ClimberSubsystem extends SubsystemBase{
    WPI_VictorSPX imparator = new WPI_VictorSPX(25);

    public ClimberSubsystem(){
    imparator.clearStickyFaults();
    imparator.setNeutralMode(NeutralMode.Brake);
    imparator.setInverted(true);
    }

    public Command climbUp(){
     return run(()-> {
            imparator.set(ControlMode.PercentOutput, 1);
        });
    }

    public Command climbDown(){
     return run(()-> {
            imparator.set(ControlMode.PercentOutput, -1);
        });
    }

    public Command stopClimb(){
     return runOnce(()-> {
            imparator.set(0);
        });
    }
}
