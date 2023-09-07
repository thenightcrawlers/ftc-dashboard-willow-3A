package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp()
public class GamePadCM extends OpMode {
    @Override
    public void init(){

    }
    @Override
    public void loop() {
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
        if(gamepad1.left_stick_y < 0) {
        telemetry.addData("Left stick y", "is negative");
    }
        telemetry.addData("Left Stick y", gamepad1.left_stick_y);
        if(gamepad1.left_stick_x < 0) {
            telemetry.addData("Left stick x", "is negative");
        }
        telemetry.addData("Left Stick x", gamepad1.left_stick_x);
        telemetry.update();
    }
}
