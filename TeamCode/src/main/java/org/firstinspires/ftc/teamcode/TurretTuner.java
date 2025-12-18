package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.controller.PIDController;

@Configurable
@TeleOp
public class TurretTuner extends LinearOpMode {

    public static int target=0;
    TelemetryManager telem;
    public void runOpMode() throws InterruptedException {

        DcMotor turretMotor;
        PIDController pid;
        pid=new PIDController(0,0,0);
        turretMotor=hardwareMap.get(DcMotor.class,Global_Configs.turretName);
        telem= PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();
        while(opModeIsActive()){
            pid.setPID(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);
            double pow=pid.calculate(turretMotor.getCurrentPosition(),target);
            turretMotor.setPower(pow);
            telem.addLine("currPos"+turretMotor.getCurrentPosition());
            telem.addLine("target"+target);
            telem.update(telemetry);
        }
    }
}
