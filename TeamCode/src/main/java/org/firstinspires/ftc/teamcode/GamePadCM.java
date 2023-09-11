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
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Left Trigger",gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Left Trigger + Right Trigger", gamepad1.left_trigger - gamepad1.right_trigger);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("B button", gamepad1.b);
        telemetry.addData("Difference between Left Joystick Y and Right Joystick Y", gamepad1.left_stick_y - gamepad1.right_stick_y);
        if(gamepad1.left_stick_y < 0) {
        telemetry.addData("Left stick y", "is negative");
    }
        if(gamepad1.left_stick_x < 0) {
            telemetry.addData("Left stick x", "is negative");
        }
        telemetry.update();
    }
}
