package org.firstinspires.ftc.teamcode.SingleUse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOP(name = "bothting")
public class MtrInit extends LinearOpMode {

    private DcMotor Rarm,Larm,SLides;

    private double Angle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {


        Rarm = hardwareMap.get(DcMotor.class,"ArmR");
        Larm = hardwareMap.get(DcMotor.class,"ArmL");
        SLides = hardwareMap.get(DcMotor.class,"Slides");

        Rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Rarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Larm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SLides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData(" Arm Ticks", Rarm.getCurrentPosition());
            telemetry.addData("Slides", SLides.getCurrentPosition());
            telemetry.update();

        }
    }

    public void TicktoAngleConverter(double tick){

        
    }

}
