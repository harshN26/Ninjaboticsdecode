package org.firstinspires.ftc.teamcode;

public class Global_Configs {
    public enum DOFStatus{ACTIVE,DISABLED}





    public final static String liftM1Name="m1Name";
    public final static String liftM2Name="m2Name";
    public final static DOFStatus liftM1Status=DOFStatus.ACTIVE; //TODO: should never be disabled - encoder pos uses this
    public final static DOFStatus liftM2Status=DOFStatus.ACTIVE;




    public final static String clawServoName="m2Name";
    public final static DOFStatus clawServoStatus=DOFStatus.ACTIVE;


}
