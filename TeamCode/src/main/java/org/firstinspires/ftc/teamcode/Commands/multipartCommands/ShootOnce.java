package org.firstinspires.ftc.teamcode.Commands.multipartCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

public class ShootOnce extends SequentialCommandGroup {
    public ShootOnce(TurretShooter shooter, ChamberSort chamber, Intake in){
        addCommands(
                new WaitUntilCommand(shooter::isInRange),
                new InstantCommand(()->chamber.update(ChamberSort.CHAMBER_STATE.UP)),
                new InstantCommand(()->in.update(Intake.INTAKE_STATE.IN)),
                new WaitUntilCommand(()-> !shooter.isInRange()),
                new InstantCommand(()->chamber.update(ChamberSort.CHAMBER_STATE.IDLE)),
                new InstantCommand(()->in.update(Intake.INTAKE_STATE.STOP))
        );
        addRequirements(shooter,chamber,in);
    }
}
