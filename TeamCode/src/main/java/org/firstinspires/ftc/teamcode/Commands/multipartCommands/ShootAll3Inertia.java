package org.firstinspires.ftc.teamcode.Commands.multipartCommands;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

public class ShootAll3Inertia extends SequentialCommandGroup {

    public ShootAll3Inertia(TurretShooter shooter, ChamberSort chamberSort, Intake intake){
        super(
                new SequentialCommandGroup(
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                        new ShootOnce(shooter,chamberSort,intake),
                        new ShootOnce(shooter,chamberSort,intake),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.STOP)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                        new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.STOP))

                )
        );


    }
}
