package org.firstinspires.ftc.teamcode.Commands.multipartCommands;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

public class ShootAll3Inertia extends SequentialCommandGroup {

    public ShootAll3Inertia(TurretShooter shooter, ChamberSort chamberSort, Intake intake){
        super(
            new SequentialCommandGroup(
                    new InstantCommand(()-> Robot_Hardware.getInstance().firing=true),
                    new WaitUntilCommand(()->shooter.inRange),
                    new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                    new InstantCommand(()->chamberSort.update(ChamberSort.CHAMBER_STATE.UP)),
                    new WaitUntilCommand(()-> !shooter.inRange),
                    new WaitUntilCommand(()->shooter.inRange),
                    new WaitUntilCommand(()-> !shooter.inRange),
                    new WaitUntilCommand(()->shooter.inRange),
                    new WaitUntilCommand(()-> !shooter.inRange),
    //                    new InstantCommand(()->chamber.update(ChamberSort.CHAMBER_STATE.IN)),
                    new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                    new InstantCommand(()-> Robot_Hardware.getInstance().firing=false)
            )
        );


    }
}
