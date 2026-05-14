package org.firstinspires.ftc.teamcode.general.examples

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.general.core.OpModeBase
import org.firstinspires.ftc.teamcode.general.hardware.SRSHub

/**
 * Bring-up benchmark for the SRSHub v1.0. All sensor reads happen through
 * a single srsHub.update() bulk transaction; [updateStats] measures that
 * round-trip cost.
 *
 * Wiring:
 *   - SRSHub → Control Hub I2C bus 3, configured as type "SRSHub" named `srsHub`
 *   - Brushland color sensor in a port labeled "I2C 1", another in "I2C 2",
 *     another in "I2C 3" on the SRSHub silkscreen
 *   - Quadrature encoder on SRSHub encoder port 1
 *   - DC motor on Control Hub motor port 0, configured as "motor0"
 *
 * Driver controls:
 *   - Left stick Y → motor0 power (forward positive; idiomatic FTC inversion)
 */
@TeleOp(name = "SRSHub: Latency Test", group = "Bring-up")
class SRSHubLatencyTest : OpModeBase() {

    private lateinit var srsHub: SRSHub
    private val colors = (1..3).associateWith { SRSHub.APDS9151() }
    private lateinit var motor: DcMotorEx

    private val updateStats = LatencyStats()
    private val loopStats = LatencyStats()
    private var lastLoopNs = 0L

    override fun configure() {
        srsHub = hardwareMap.get(SRSHub::class.java, "srsHub")
        val config = SRSHub.Config()
        for ((bus, sensor) in colors) {
            config.addI2CDevice(bus, sensor)
        }
        config.setEncoder(ENCODER_PORT, SRSHub.Encoder.QUADRATURE)
        srsHub.init(config)

        motor = hardwareMap.get(DcMotorEx::class.java, "motor0").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }
    }

    override fun onStart() {
        lastLoopNs = System.nanoTime()
    }

    override fun onLoop() {
        val t0 = System.nanoTime()
        srsHub.update()
        val t1 = System.nanoTime()
        updateStats.record(t1 - t0)

        val power = -driver.leftStickY
        motor.power = power.toDouble()

        val encoder = srsHub.readEncoder(ENCODER_PORT)

        val now = System.nanoTime()
        loopStats.record(now - lastLoopNs)
        lastLoopNs = now

        telemetryBag.section("SRSHub") {
            put("ready", srsHub.ready())
            put("disconnected", srsHub.disconnected())
            put("update() µs", updateStats.summary())
        }
        telemetryBag.section("Encoder port $ENCODER_PORT") {
            put("position", encoder.position)
            put("velocity", encoder.velocity)
        }
        telemetryBag.section("Motor (motor0)") {
            put("power", power.toDouble(), decimals = 2)
        }
        for ((bus, sensor) in colors) {
            telemetryBag.section("Color bus $bus") {
                put("disconnected", sensor.disconnected)
                put("r/g/b", "${sensor.red}/${sensor.green}/${sensor.blue}")
                put("infrared", sensor.infrared)
                put("proximity", sensor.proximity.toInt())
            }
        }
        telemetryBag.section("Loop") {
            put("period µs", loopStats.summary())
            put("Hz", if (loopStats.avgNs > 0) 1e9 / loopStats.avgNs else 0.0, decimals = 1)
        }
    }

    private class LatencyStats {
        var minNs = Long.MAX_VALUE
        var maxNs = 0L
        var sumNs = 0L
        var count = 0L

        val avgNs: Double get() = if (count == 0L) 0.0 else sumNs.toDouble() / count

        fun record(ns: Long) {
            if (ns < minNs) minNs = ns
            if (ns > maxNs) maxNs = ns
            sumNs += ns
            count++
        }

        fun summary(): String {
            if (count == 0L) return "n/a"
            return "min=%d avg=%.0f max=%d".format(minNs / 1000, avgNs / 1000, maxNs / 1000)
        }
    }

    companion object {
        private const val ENCODER_PORT = 1
    }
}
