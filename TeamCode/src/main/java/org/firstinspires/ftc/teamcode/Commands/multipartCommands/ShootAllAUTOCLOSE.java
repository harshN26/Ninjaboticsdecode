package org.firstinspires.ftc.teamcode.Commands.multipartCommands;



import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootOnce;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

public class ShootAllAUTOCLOSE extends SequentialCommandGroup {

    public ShootAllAUTOCLOSE(TurretShooter shooter, ChamberSort chamberSort, Intake intake){
        super(
                new SequentialCommandGroup(
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                    new InstantCommand(()-> chamberSort.update(ChamberSort.CHAMBER_STATE.UP)),
                        new InstantCommand(()-> intake.update(Intake.INTAKE_STATE.IN)),
                        new WaitCommand(500),
                    new ShootOnce(shooter,chamberSort,intake),
                    new WaitCommand(500),
                    new ShootOnce(shooter,chamberSort,intake),
                        new WaitCommand(500),
                    new ShootOnce(shooter,chamberSort,intake),
                    new InstantCommand(()->shooter.update(TurretShooter.shooterState.STOP)),
                    new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                    new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.STOP))
                )
        );


    }
}
