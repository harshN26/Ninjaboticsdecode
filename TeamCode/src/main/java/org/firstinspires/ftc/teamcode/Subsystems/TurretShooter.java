package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@Configurable
public class TurretShooter extends SubsystemBase {
    Robot_Hardware robot;


    public enum shooterState {FIRE, IDLE, STOP, RESET, RESET_COMPLETE}

    public shooterState state = shooterState.IDLE;

    TelemetryManager telemetry;

    private double currentRPM_shooter;
    private double targetRPM_shooter;

    public boolean inRange;

    private double hoodPos;
    public static double targetHoodPos;

    private double shooterPow=0.0;





    public double tol_turret;
    public int turretTarget;
    private int turretCurrPos;
    Telemetry telem;
    private final ElapsedTime timer = new ElapsedTime();


    PIDController pidTurret;



    public TurretShooter(Robot_Hardware hardware, Telemetry telemetry) {
        robot = hardware;
        targetRPM_shooter = 0;
        targetHoodPos=0;




        turretTarget=0;
        tol_turret=Constants.tolerance_turret;

        telem = telemetry;
        this.telemetry=PanelsTelemetry.INSTANCE.getTelemetry();
        pidTurret=new PIDController(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);
        pidTurret.setPID(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);

    }




    public void set_target_turret(int ticks){
            turretTarget=ticks;
    }

    public void set_target_turret(double radians){

    }
    public void set_target_tolerance(int tol){
        tol_turret=tol;
    }

    public int get_target_ticks(){
        return turretTarget;
    }

    public int getTurretPos(){
        return turretCurrPos;
    }


    public double[] getPIDCoeffsTurret() {
        return Constants.pidCoeffs_turret;
    }

    public double get_tolTurret() {
        return tol_turret;
    }

    public void resetEncoderTurret() {
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void updateTurretPos(){
        turretCurrPos=robot.turret.getCurrentPosition();
    }






    public void set_targetRPM_shooter(double new_target) {
        targetRPM_shooter = new_target;
    }


    private void updateCurrentSpeedShooter() {
        double ticksPerSecond = robot.shooterM1.getVelocity();
        currentRPM_shooter = (ticksPerSecond / Constants.TICKS_PER_REV_Shooter) * 60.0;
    }



    public double getCurrentRPMShooter() {
        return currentRPM_shooter;
    }

    public double getTargetRPMShooter() {
        return targetRPM_shooter;
    }


    public double get_tolShooter() {
        return Constants.tolerance_shooter;
    }







    public void setHoodTarget(double pos) {
        targetHoodPos = pos;
    }
    public void updateHoodPos() {
        hoodPos=robot.hood1.getPosition();
    }






    public boolean isInRange() {
        boolean rpmOK = (currentRPM_shooter >= targetRPM_shooter - Constants.tolerance_shooter && currentRPM_shooter <= targetRPM_shooter + Constants.tolerance_shooter);
        boolean hoodOK = Math.abs(targetHoodPos - hoodPos) <= Constants.hoodTolerance;
        boolean turretOK=Math.abs(turretTarget-turretCurrPos)<=Constants.tolerance_turret;
        inRange = rpmOK&&hoodOK&&turretOK&&state==shooterState.FIRE;
        //inRange=rpmOK&&hoodOK&&turretOK;
        return inRange;
    }




    public void update(shooterState newState) {
        state = newState;
        inRange = false;
    }




    private double[] calculateShot(double xTarget, double yTarget) {
        // Horizontal displacement with velocity compensation
        double dx = xTarget - robot.x - robot.xVelo;
        double dy = yTarget - robot.y - robot.yVelo;

        // Distance to target
        double horizontalDistance = Math.hypot(dx, dy);

        // Turret target angle in radians (0 = back, π = front)
        double targetAngleRad = Math.atan2(dy, dx) + Math.PI;

        // Convert turret angle to motor ticks with gear ratio (3:1)
        double motorRevs = (targetAngleRad / (2.0 * Math.PI)) * 3.0; // 3 motor revs per turret rev
        int targetTicks = (int)(motorRevs * Constants.TICKS_PER_REV_Turret);

        // Shortest-path turret rotation
        int deltaTicks = targetTicks - turretCurrPos;
        int fullTurretTicks = (int)(Constants.TICKS_PER_REV_Turret * 3.0); // full turret rotation in motor ticks
        if (deltaTicks > fullTurretTicks / 2) {
            deltaTicks -= fullTurretTicks;
        } else if (deltaTicks < -fullTurretTicks / 2) {
            deltaTicks += fullTurretTicks;
        }

        int finalTurretTarget = turretCurrPos + deltaTicks;

        // Initial launch angle guess
        double theta = Math.atan(Constants.dZ / horizontalDistance);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // Ball velocity using projectile equation
        double denom = 2.0 * cosTheta * cosTheta * (horizontalDistance * tanTheta - Constants.dZ);
        if (denom <= 0) denom = 1e-6; // prevent division by zero
        double vBall = Math.sqrt(Constants.g * horizontalDistance * horizontalDistance / denom);

        // Convert velocity to RPM
        double rawRPM = (vBall / (2.0 * Math.PI * Constants.shooterWheelRadius)) * 60.0;

        // Apply global tuning factor
        rawRPM *= Constants.RPM_TUNING_FACTOR;

        // Apply distance-based RPM bias
        rawRPM += closeRangeRPMBias(horizontalDistance);

        // Enforce minimum and maximum RPM
        rawRPM = Math.max(Constants.MIN_WHEEL_RPM, Math.min(rawRPM, Constants.MAX_WHEEL_RPM));

        // Recompute velocity from final RPM for accurate hood calculation
        vBall = (rawRPM / 60.0) * 2.0 * Math.PI * Constants.shooterWheelRadius;

        // Solve launch angle with final velocity
        double v2 = vBall * vBall;
        double g = Constants.g;
        double x = horizontalDistance;
        double z = Constants.dZ;
        double underSqrt = v2 * v2 - g * (g * x * x + 2.0 * z * v2);
        if (underSqrt < 0) underSqrt = 0;
        theta = Math.atan((v2 - Math.sqrt(underSqrt)) / (g * x));

        // Hood position normalized (0–1)
        double hoodPos = Math.max(0.0, Math.min(1.0, (theta - Math.toRadians(15.0)) / Math.toRadians(30.0)));

        // Effective RPM after factoring system efficiency
        double effectiveRPM = Math.min(rawRPM * Constants.EFFECTIVE_RPM_FACTOR, Constants.MAX_WHEEL_RPM);

        // Return: raw RPM, effective RPM, hood position, turret target (motor ticks)
        return new double[]{rawRPM, effectiveRPM, hoodPos, finalTurretTarget};
    }



    private double closeRangeRPMBias(double distance) {
        return Constants.CLOSE_RPM_BIAS *
                Math.exp(-distance / Constants.CLOSE_BIAS_DECAY);
    }

    private double[] calculateShot2(double xTarget, double yTarget){
        double dy = yTarget - robot.y-robot.yVelo;
        double dx = xTarget - robot.x-robot.xVelo;
        double horizontalDistance = Math.hypot(dy, dx);

        double fieldAngle = Math.atan2(dy, dx);
        double robotdAngle = (fieldAngle - robot.heading);

        double turretCurrAngle = fieldAngle - robotdAngle - Math.PI;
        if (turretCurrAngle < Math.toRadians(-180)) {
            turretCurrAngle += 2 * Math.PI;
        }

        double dtheta1 = (turretCurrAngle - fieldAngle) % (2 * Math.PI);
        double dtheta2 = ((turretCurrAngle + 2 * Math.PI) - fieldAngle) % (2 * Math.PI);
        double dtheta = Math.abs(dtheta1) > Math.abs(dtheta2) ? dtheta2 : dtheta1;

        double motorRevs = (dtheta / (2.0 * Math.PI)) * 3.0;
        int targetTicks = (int)(motorRevs * Constants.TICKS_PER_REV_Turret);
        targetTicks = Math.max(-600, Math.min(600, targetTicks));

        // ---------------- BALLISTICS (RAW RPM FIXED) ----------------

        double theta = Math.toRadians(25.0);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        double term = horizontalDistance * tanTheta - Constants.dZ;

        double rawRPM;

        if (term > 0.05) {
            // valid projectile solution
            double denom = 2.0 * cosTheta * cosTheta * term;
            double vBall = Math.sqrt(Constants.g * horizontalDistance * horizontalDistance / denom);
            rawRPM = (vBall / (2.0 * Math.PI * Constants.shooterWheelRadius)) * 60.0;
        } else {
            // close range fallback (physically correct)
            rawRPM =
                    Constants.MIN_WHEEL_RPM +
                            horizontalDistance * Constants.CLOSE_RANGE_RPM_SLOPE;
        }

        rawRPM *= Constants.RPM_TUNING_FACTOR;
        rawRPM += closeRangeRPMBias(horizontalDistance);

        telem.addLine("rpm: " + rawRPM);

        rawRPM = Math.max(Constants.MIN_WHEEL_RPM,
                Math.min(rawRPM, Constants.MAX_WHEEL_RPM));

        // ---------------- REMAINDER UNCHANGED ----------------

        double vBall =
                (rawRPM / 60.0) * 2.0 * Math.PI * Constants.shooterWheelRadius;

        double v2 = vBall * vBall;
        double g = Constants.g;
        double x = horizontalDistance;
        double z = Constants.dZ;

        double underSqrt = v2 * v2 - g * (g * x * x + 2.0 * z * v2);
        if (underSqrt < 0) underSqrt = 0;

        theta = Math.atan((v2 - Math.sqrt(underSqrt)) / (g * x));

        double hoodPos = Math.max(0.0,
                Math.min(1.0,
                        (theta - Math.toRadians(15.0)) / Math.toRadians(30.0)));

        double effectiveRPM =
                Math.min(rawRPM * Constants.EFFECTIVE_RPM_FACTOR,
                        Constants.MAX_WHEEL_RPM);

        return new double[]{rawRPM, effectiveRPM, hoodPos, targetTicks};
    }

    private double[] calculateShot1(double xTarget, double yTarget) {
        // ---------------- DISPLACEMENT ----------------
        double dx = xTarget - robot.x - robot.xVelo;
        double dy = yTarget - robot.y - robot.yVelo;
        double horizontalDistance = Math.hypot(dx, dy);

        // ---------------- CONSTANTS ----------------
        final double GEAR = 3.0; // motor revs per turret rev
        final double TICKS_PER_REV_MOTOR = Constants.TICKS_PER_REV_Turret; // 384.5
        final double FULL_MOTOR_TICKS = TICKS_PER_REV_MOTOR * GEAR;
        final double HALF_FULL_TICKS = FULL_MOTOR_TICKS / 2.0;

        // ---------------- TURRET ANGLE (radians) ----------------
        // Field angle: absolute angle from robot to target in field coordinates
        double fieldAngle = Math.atan2(dy, dx);

        // Robot forward is initially at 90° (π/2), so robot.heading starts at π/2
        // Turret angle relative to robot forward (0 = forward, positive = CCW)
        double turretAngleRelative = fieldAngle - robot.heading;

        // Normalize to [-π, π]
        while (turretAngleRelative > Math.PI) turretAngleRelative -= 2.0 * Math.PI;
        while (turretAngleRelative < -Math.PI) turretAngleRelative += 2.0 * Math.PI;

        // Convert to encoder ticks
        // When turret is at 0° relative (pointing forward with robot), it's physically at 270° + robot.heading
        // Encoder zero is at physical 270° (robot's initial backward position)
        // So: encoder_position = (turret_relative_angle) * (gear_ratio) * (ticks_per_motor_rev) / (2π)
        // But we need to account for the fact that encoder=0 when turret points backward (180° from forward)

        // Turret relative angle where 0 = forward, π = backward
        // Encoder 0 = backward position
        // So encoder reading = (turret_relative - π) converted to ticks
        double turretAngleForEncoder = turretAngleRelative - Math.PI;

        // Convert to motor ticks
        double motorRevsRaw = turretAngleForEncoder / (2.0 * Math.PI) * GEAR;
        double rawTicks = motorRevsRaw * TICKS_PER_REV_MOTOR;

        // Shortest-path delta (signed) relative to current encoder position
        double deltaTicks = rawTicks - turretCurrPos;

        // Wrap delta into [-HALF_FULL_TICKS, +HALF_FULL_TICKS]
        if (deltaTicks > HALF_FULL_TICKS) {
            deltaTicks -= FULL_MOTOR_TICKS;
        } else if (deltaTicks < -HALF_FULL_TICKS) {
            deltaTicks += FULL_MOTOR_TICKS;
        }

        int finalTargetTicks = (int) Math.round(turretCurrPos + deltaTicks);

        // Clamp to soft limits [-600, +600]
        if (finalTargetTicks > 600) finalTargetTicks = 600;
        if (finalTargetTicks < -600) finalTargetTicks = -600;

        telem.addData("Field Angle (deg)", Math.toDegrees(fieldAngle));
        telem.addData("Robot Heading (deg)", Math.toDegrees(robot.heading));
        telem.addData("Turret Relative (deg)", Math.toDegrees(turretAngleRelative));
        telem.addData("Target Ticks", finalTargetTicks);
        telem.addData("Current Ticks", turretCurrPos);
        telem.addData("Delta Ticks", deltaTicks);

        // ---------------- BALLISTICS (unchanged) ----------------
        double theta = Math.atan(Constants.dZ / horizontalDistance);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        double denom = 2.0 * cosTheta * cosTheta * (horizontalDistance * tanTheta - Constants.dZ);
        if (denom <= 0) denom = 1e-6;

        double vBall = Math.sqrt(Constants.g * horizontalDistance * horizontalDistance / denom);
        double rawRPM = (vBall / (2.0 * Math.PI * Constants.shooterWheelRadius)) * 60.0;

        rawRPM *= Constants.RPM_TUNING_FACTOR;
        rawRPM += closeRangeRPMBias(horizontalDistance);
        rawRPM = Math.max(Constants.MIN_WHEEL_RPM, Math.min(rawRPM, Constants.MAX_WHEEL_RPM));

        vBall = (rawRPM / 60.0) * 2.0 * Math.PI * Constants.shooterWheelRadius;

        double v2 = vBall * vBall;
        double g = Constants.g;
        double x = horizontalDistance;
        double z = Constants.dZ;

        double underSqrt = v2 * v2 - g * (g * x * x + 2.0 * z * v2);
        if (underSqrt < 0) underSqrt = 0;

        theta = Math.atan((v2 - Math.sqrt(underSqrt)) / (g * x));

        double hoodPos = Math.max(0.0,
                Math.min(1.0, (theta - Math.toRadians(15.0)) / Math.toRadians(30.0)));

        double effectiveRPM = Math.min(rawRPM * Constants.EFFECTIVE_RPM_FACTOR, Constants.MAX_WHEEL_RPM);

        return new double[]{rawRPM, effectiveRPM, hoodPos, finalTargetTicks};
    }







    public void loop() {

        updateCurrentSpeedShooter();
        updateHoodPos();
        updateTurretPos();
        inRange=isInRange();

        pidTurret.setPID(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);

        double [] results= calculateShot2(robot.goal.getX(),robot.goal.getY());
        switch (state) {
            case FIRE:
                // active tracking and everything, we are ready for shooting and waiting for balls to enter

//                set_targetRPM_shooter(results[0]);
                set_targetRPM_shooter((int)results[0]);
//                setHoodTarget(results[2]);
                setHoodTarget((int)results[2]);
//                set_target_turret((int)results[3]);
                set_target_turret((int)results[3]);

                break;
            case IDLE:
                // get turret to be as close to 0 as possible and shooter to current-appropriate speed
                set_targetRPM_shooter(Constants.ShooterIdleRPM);
                setHoodTarget(Constants.hoodResetPos);
                set_target_turret(0);
                break;
            case STOP:
                // complete standstill, analysing if something is wrong or on low voltage
                set_targetRPM_shooter(Constants.ShooterStopRPM);
                setHoodTarget(Constants.hoodMaxPos);
                set_target_turret(0);
                break;
            case RESET:
                // something has gone wrong, try to fix
                set_targetRPM_shooter(Constants.ShooterResetRPM);
                setHoodTarget(Constants.hoodResetPos);
                set_target_turret(0);
                if(robot.turretZero.getState()){
                    update(shooterState.RESET_COMPLETE);
                }
                break;
            case RESET_COMPLETE:
                break;
        }





        double turretPow;

        turretPow=pidTurret.calculate(turretCurrPos,turretTarget);
        shooterPow= Constants.shooter_kp * (targetRPM_shooter-currentRPM_shooter) + Constants.shooter_kv * targetRPM_shooter + Math.signum(targetRPM_shooter-currentRPM_shooter) * Constants.shooter_ks;
        robot.shooterM1.setPower(shooterPow);
//        robot.shooterM1.setPower(0.9);
        if (Global_Configs.shooterM2Status == Global_Configs.DOFStatus.ACTIVE)
            robot.shooterM2.setPower(shooterPow);
//            robot.shooterM2.setPower(0.9);
        else {
            robot.shooterM2.setPower(0);
        }

        if (Global_Configs.turretStatus == Global_Configs.DOFStatus.ACTIVE)
            robot.turret.setPower(turretPow);
        else {
            robot.turret.setPower(0);
        }


        if(Global_Configs.hood1Status== Global_Configs.DOFStatus.ACTIVE) {
            robot.hood1.setPosition(targetHoodPos);
        }else{
            robot.hood1.setPosition(Constants.hoodResetPos);
        }

        if(Global_Configs.hood2Status== Global_Configs.DOFStatus.ACTIVE) {
            robot.hood2.setPosition(targetHoodPos);
        }else{
            robot.hood2.setPosition(Constants.hoodResetPos);
        }

        telem(); 
        telemP();
    }

    public void telem() {

        telem.addLine("target RPM: " + targetRPM_shooter);
        telem.addLine("current RPM: " + currentRPM_shooter);
        telem.addLine("target turret Pos: " + turretTarget);
        telem.addLine("current turret Pos: " + turretTarget);
        telem.addLine("target hood Pos: " + targetHoodPos);
        telem.addLine("current hood Pos: " + targetHoodPos);

        telem.addLine("shooter in range: " + inRange);

//        telem.addLine("shooter state: " + state);
    }
    public void telemP() {

        telemetry.addLine("target RPM: "+targetRPM_shooter);
        telemetry.addLine("current RPM: " + currentRPM_shooter);
        telemetry.addLine("target turret Pos: " + turretTarget);
        telemetry.addLine("current turret Pos: " + turretTarget);
        telemetry.addLine("target hood Pos: " + targetHoodPos);
        telemetry.addLine("current hood Pos: " + targetHoodPos);

        telemetry.addLine("shooter in range: " + inRange);
        telemetry.update(telem);

//        telem.addLine("shooter state: " + state);
    }


}
