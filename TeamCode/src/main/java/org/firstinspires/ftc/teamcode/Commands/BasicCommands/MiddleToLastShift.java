package org.firstinspires.ftc.teamcode.Commands.BasicCommands;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAll3Inertia;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootOnce;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

public class MiddleToLastShift extends SequentialCommandGroup {

    public MiddleToLastShift(TurretShooter shooter, ChamberSort chamberSort, Intake intake){
        super(
                new SequentialCommandGroup(
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                        new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->chamberSort.updateSort(ChamberSort.SORT_STATE.SHIFTIN)),
                        new WaitCommand(100),
                        new InstantCommand(()->chamberSort.updateSort(ChamberSort.SORT_STATE.HOLD)),
                        new ShootOnce(shooter,chamberSort,intake),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                        new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.STOP)),
                        new InstantCommand(()->chamberSort.updateSort(ChamberSort.SORT_STATE.THROUGHPUT)),
                        new WaitCommand(100),
                        new ShootAll3Inertia(shooter,chamberSort,intake),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                        new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.STOP))

                )
        );


    }
}
