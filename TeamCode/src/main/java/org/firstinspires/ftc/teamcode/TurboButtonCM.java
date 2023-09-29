package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp()
public class TurboButtonCM extends OpMode {
    private DcMotor Motor1;
    @Override
    public void init() {
        telemetry.addData("Left Stick x", gamepad1.left_stick_x);
        telemetry.addData("Left Stick y", gamepad1.left_stick_y);
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
    }
    @Override
    public void loop() {

        double fwdSpeed = gamepad1.left_stick_y;
        double ySpeed = gamepad1.left_stick_y;;
        double xSpeed = gamepad1.left_stick_x;

        if(gamepad1.a) {
            fwdSpeed *= .5;
        }
        if (gamepad1.a) {
            fwdSpeed *= 1;
        }
        if(!gamepad1.a) {
            ySpeed = gamepad1.left_stick_x;
            xSpeed = gamepad1.left_stick_y;
        }
        if(gamepad1.b) {
            Motor1.setPower(1);
        }

    }
}
