///




























package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="MenaceBotCM")
@Config
//@Disabled
public class MenaceBotCM extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx armDrive = null;
    private Servo leftservo = null;
    private Servo rightservo = null;
    public static int right_claw_open = 70;
    public static int right_claw_close = -40;
    public static int left_claw_open = -70;
    public static int left_claw_close = 40;
    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor;
    public static double speed = 0.6;
    //
    //

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotorEx.class, "arm_motor");
        leftservo = hardwareMap.get(Servo.class,"left_servo");
        rightservo = hardwareMap.get(Servo.class, "right_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

FtcDashboard dashboard= FtcDashboard.getInstance();
telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                armDrive.setTargetPosition(-650);
                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armDrive.setVelocity(200);
            }
            if (gamepad1.dpad_down) {
                armDrive.setTargetPosition(-100);
                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armDrive.setVelocity(200);
            }

            if(gamepad1.b){
                    int position = armDrive.getCurrentPosition();
                    int newPosition = position - 10;
                    armDrive.setTargetPosition(newPosition);
                    armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armDrive.setPower(0.5);
                }
            if(gamepad1.x){
                int position = armDrive.getCurrentPosition();
                int newPosition = position + 10;
                armDrive.setTargetPosition(newPosition);
                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armDrive.setPower(0.5);
            }


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            double turn = -gamepad1.right_stick_x;
            double drive  =  -gamepad1.left_stick_x;
            leftPower    = Range.clip(drive + turn, -speed, speed) ;
            rightPower   = Range.clip(drive - turn, -speed, speed) ;

            if(gamepad1.right_stick_button){
                if (speed < 1){
                    speed = speed + 0.2;
                }
            }
            if(gamepad1.left_stick_button){
                if (speed > .2) {
                    speed = speed - 0.2;
                }
            }
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            if(gamepad1.y){
                if(distanceSensor.getDistance(DistanceUnit.INCH)<1){
                    rightservo.setPosition(right_claw_close);
                    leftservo.setPosition(left_claw_close);
                }
            }





            if(gamepad1.left_bumper){
                leftservo.setPosition(left_claw_open);
                rightservo.setPosition(right_claw_open);
            }
            if(gamepad1.right_bumper){
                leftservo.setPosition(left_claw_close);
                rightservo.setPosition(right_claw_close);
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.addData("Arm Position", armDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
