package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.hypot
import org.firstinspires.ftc.teamcode.core.pathing.chaseTarget
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Validates `chaseTarget` without needing a vision pipeline wired up.
 *
 * A "virtual ball" lives in field coordinates. The driver moves it around
 * with the right stick (forward/back/left/right in field frame) and the
 * robot continuously chases it via Pedro's holdPoint, retargeted every
 * tick by the chaseTarget command.
 *
 * Controls:
 *  - Right stick → push the virtual ball around the field
 *  - A          → re-place the ball [initialBallOffsetIn] inches in front
 *                 of the robot's current pose (good for resetting if it
 *                 drifts off-field)
 *
 * What to look for on the dashboard / Driver Station:
 *  - `dist` should shrink to ~0 when you let go of the stick
 *  - `dist` should track at roughly a constant lag when you drag the
 *    stick continuously — that lag is your tracking error and indicates
 *    whether the translation PIDs need tuning for moving setpoints
 *  - If the robot overshoots / oscillates after you release the stick,
 *    Pedro's translation I term is winding up — reduce it in Constants
 */
@TeleOp(name = "Test: Chase Target", group = "Test")
@Configurable
class ChaseTestTeleOp : OpModeBase() {

    companion object {
        /** Max speed the virtual ball moves under full stick deflection (inches/sec). */
        @JvmField var ballSpeedInPerSec: Double = 24.0

        /** Where to place the ball relative to robot pose when A is pressed. */
        @JvmField var initialBallOffsetIn: Double = 24.0
    }

    private lateinit var drive: MecanumDriveSubsystem
    private var ballPose: Pose = Pose()
    private var lastTickMs: Long = 0L

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
    }

    override fun onStart() {
        ballPose = ballInFrontOfRobot()
        lastTickMs = System.currentTimeMillis()
        Scheduler.schedule(chaseTarget(drive, { ballPose }))
    }

    override fun onLoop() {
        val now = System.currentTimeMillis()
        val dt = (now - lastTickMs) / 1000.0
        lastTickMs = now

        // Right stick moves the ball in field frame.
        // GamepadEx already inverts Y so positive = stick up = +X (forward).
        val dx = driver.rightStickY * ballSpeedInPerSec * dt
        val dy = -driver.rightStickX * ballSpeedInPerSec * dt
        ballPose = Pose(ballPose.x + dx, ballPose.y + dy, ballPose.heading)

        if (driver.aPressed) ballPose = ballInFrontOfRobot()

        val robotPose = drive.pose
        val dist = hypot(ballPose.x - robotPose.x, ballPose.y - robotPose.y)

        telemetryBag.section("Chase Test") {
            put("ball", ballPose)
            put("robot", robotPose)
            put("dist", dist, decimals = 2)
        }
        telemetryBag.section("Drive") {
            put("mode", drive.mode.name)
            put("isMoving", drive.isMoving)
            put("velocity", drive.velocity)
        }
        telemetryBag.section("Robot") {
            put("loopHz", robot.loopHz, decimals = 1)
        }
    }

    private fun ballInFrontOfRobot(): Pose {
        val p = drive.pose
        // Place the ball straight ahead in field frame (along +X for heading=0).
        // For non-zero starting heading you may want to rotate this; for a
        // bring-up test, field-frame +X is fine.
        return Pose(p.x + initialBallOffsetIn, p.y, p.heading)
    }
}
