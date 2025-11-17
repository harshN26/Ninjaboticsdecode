package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Global_Configs {
    public enum DOFStatus{ACTIVE,DISABLED}

    public final static String shooterM1Name="m1Name";
    public final static String shooterM2Name="m2Name";
    public final static DOFStatus shooterM1Status=DOFStatus.ACTIVE; //TODO: should never be disabled - encoder pos uses this
    public final static DOFStatus shooterM2Status=DOFStatus.ACTIVE;

    public final static String turretName="turret";
    public final static DOFStatus turretStatus=DOFStatus.ACTIVE;
    public final static String intakeName="intake";
    public final static DOFStatus intakeStatus=DOFStatus.ACTIVE;

    public final static String sort1Name="sort1";
    public final static DOFStatus sort1Status=DOFStatus.ACTIVE;
    public final static String sort2Name="sort2";
    public final static DOFStatus sort2Status=DOFStatus.ACTIVE;
    public final static String pushName="push";
    public final static DOFStatus pushStatus=DOFStatus.ACTIVE;
    public final static String rampName="ramp";
    public final static DOFStatus rampStatus=DOFStatus.ACTIVE;

    public final static String flickName="flick";
    public final static DOFStatus flickStatus=DOFStatus.ACTIVE;
    public final static String hood1Name="hood1";
    public final static DOFStatus hood1Status=DOFStatus.ACTIVE;
    public final static String hood2Name="hood2";
    public final static DOFStatus hood2Status=DOFStatus.ACTIVE;


    public final static String ledsName="leds";


}
