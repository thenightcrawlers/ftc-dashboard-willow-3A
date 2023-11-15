package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MenaceBot2CM")
@Config
//@Disabled
public class MenaceBot2CM extends LinearOpMode {

    //Declare OpMode Members
    //Private
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx armMotor = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    //Public
    public static int rightClaw_open = 70;
    public static int rightClaw_close = -40;
    public static int leftClaw_open = -70;
    public static int leftClaw_close = 40;
    public static int armUp = -500;
    public static int armDown = -60;
    public static int object = 1;
    public static double speed = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Hardware Map
        //Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //Servos
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        //Sensors
        colorSensor = hardwareMap.get(ColorSensor.class,"color_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Drive Directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        //Stops and Resets Encoder
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            //Driving Variables
            //Power
            double leftPower;
            double rightPower;
            //Movement
            double turn = gamepad1.left_stick_x;
            double drive = gamepad1.left_stick_y;

            leftPower = Range.clip(drive + turn, -speed, speed);
            rightPower = Range.clip(drive - turn, -speed, speed);

            //Sends Power to Wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //Arm Lift to Position (Using Encoder)
            if (gamepad1.dpad_up) {
                armMotor.setTargetPosition(armUp);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(200);
            }

            //Arm Lower to Position (Using Encoder)
            if (gamepad1.dpad_down) {
                armMotor.setTargetPosition(armDown);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(200);
            }

            //Arm Lift Position
            if(gamepad1.b) {
                int temp = armMotor.getCurrentPosition();
                int newPosition = temp - 10;
                armMotor.setTargetPosition(newPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }

            //Arm Lower Position
            if(gamepad1.x) {
                int temp = armMotor.getCurrentPosition();
                int newPosition = temp + 10;
                armMotor.setTargetPosition(newPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }

            //Claw Code
            //Claw Open
            if (gamepad1.left_bumper) {
                rightServo.setPosition(leftClaw_open);
                leftServo.setPosition(rightClaw_open);
            }
            //Claw Close
            if (gamepad1.right_bumper) {
                rightServo.setPosition(leftClaw_close);
                leftServo.setPosition(rightClaw_close);
            }

            //Distance Sensor Claw Close
            if(gamepad1.y) {
                if (distanceSensor.getDistance(DistanceUnit.INCH) < object) {
                    rightServo.setPosition(leftClaw_close);
                    leftServo.setPosition(rightClaw_close);
                }
            }

            //Drive and Turn Speed
            //Speed Max
            if(gamepad1.left_stick_button){
                speed = 1.00;
            }
            //Speed Middle
            if(gamepad1.a) {
                speed = 0.50;
            }
            //Speed Min
            if(gamepad1.right_stick_button){
                speed = 0.25;
            }
        }

        //Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Arm Position: ", armMotor.getCurrentPosition());
        telemetry.addData("Driving/Turning Power: ", speed);
        telemetry.addData("Object Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.update();
    }
}