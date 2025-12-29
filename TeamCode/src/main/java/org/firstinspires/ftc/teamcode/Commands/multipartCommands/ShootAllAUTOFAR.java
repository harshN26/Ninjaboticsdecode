package org.firstinspires.ftc.teamcode.Commands.multipartCommands;


import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

public class ShootAllAUTOFAR extends SequentialCommandGroup {

    public ShootAllAUTOFAR(TurretShooter shooter, ChamberSort chamberSort, Intake intake){
        super(
                new SequentialCommandGroup(
                    new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOFAR)),
                    new ShootOnce(shooter,chamberSort,intake),
                    new ShootOnce(shooter,chamberSort,intake),
                    new ShootOnce(shooter,chamberSort,intake),
                    new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                    new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                    new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.STOP))
                )
        );


    }
}
