package org.firstinspires.ftc.teamcode.general.examples

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.general.core.OpModeBase
import org.firstinspires.ftc.teamcode.general.hardware.SRSHub

/**
 * Side-by-side: 3 color sensors hanging off the SRSHub vs 3 color sensors
 * wired directly to Control Hub I2C buses 0/1/2. The SRSHub bulk-read
 * always runs every loop; the direct reads are gated on a gamepad toggle
 * so you can watch the loop-time delta in real time.
 *
 *  - Driver A: toggles direct-I2C reads on/off (also resets stats)
 *  - Driver Y: resets stats without changing the toggle
 *
 * Wiring:
 *  - SRSHub → Control Hub I2C bus 3, configured as type "SRSHub" named `srsHub`
 *  - Brushland sensor on each of SRS I2C 1, 2, 3 (not in the FTC config)
 *  - Brushland sensor on each of Control Hub I2C 0, 1, 2, configured as
 *    type "REV Color Sensor V3" named `direct0`, `direct1`, `direct2`
 *
 * Note: Control Hub I2C bus 0 also hosts the built-in IMU at 0x28. The
 * color sensor at 0x52 doesn't conflict, but the IMU's own polling will
 * share that bus's bandwidth.
 */
@TeleOp(name = "Compare: SRSHub vs Direct I2C", group = "Bring-up")
class ColorSensorComparison : OpModeBase() {

    private lateinit var srsHub: SRSHub
    private val srsColors = (1..3).associateWith { SRSHub.APDS9151() }
    private lateinit var direct: List<RevColorSensorV3>

    private val srsStats = LatencyStats()
    private val directStats = LatencyStats()
    private val loopStats = LatencyStats()
    private var lastLoopNs = 0L
    private var readDirect = false

    override fun configure() {
        srsHub = hardwareMap.get(SRSHub::class.java, "srsHub")
        val config = SRSHub.Config()
        for ((bus, sensor) in srsColors) config.addI2CDevice(bus, sensor)
        srsHub.init(config)

        direct = listOf("direct0", "direct1", "direct2")
            .map { hardwareMap.get(RevColorSensorV3::class.java, it) }
    }

    override fun onStart() {
        lastLoopNs = System.nanoTime()
    }

    override fun onLoop() {
        if (driver.aPressed) {
            readDirect = !readDirect
            resetStats()
        }
        if (driver.yPressed) {
            resetStats()
        }

        val t0 = System.nanoTime()
        srsHub.update()
        val t1 = System.nanoTime()
        srsStats.record(t1 - t0)

        if (readDirect) {
            val t2 = System.nanoTime()
            for (s in direct) {
                s.argb()
            }
            val t3 = System.nanoTime()
            directStats.record(t3 - t2)
        }

        val now = System.nanoTime()
        loopStats.record(now - lastLoopNs)
        lastLoopNs = now

        telemetryBag.section("Mode") {
            put("readDirect (A toggle)", readDirect)
            put("samples", loopStats.count)
        }
        telemetryBag.section("SRSHub (3 sensors, bulk)") {
            put("update() µs", srsStats.summary())
            put("ready", srsHub.ready())
        }
        telemetryBag.section("Direct I2C (3 sensors, sequential)") {
            put("argb() x3 µs", if (readDirect) directStats.summary() else "OFF")
        }
        telemetryBag.section("Loop") {
            put("period µs", loopStats.summary())
            put("Hz", if (loopStats.avgNs > 0) 1e9 / loopStats.avgNs else 0.0, decimals = 1)
        }
    }

    private fun resetStats() {
        srsStats.reset()
        directStats.reset()
        loopStats.reset()
        lastLoopNs = System.nanoTime()
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

        fun reset() {
            minNs = Long.MAX_VALUE; maxNs = 0L; sumNs = 0L; count = 0L
        }

        fun summary(): String {
            if (count == 0L) return "n/a"
            return "min=%d avg=%.0f max=%d".format(minNs / 1000, avgNs / 1000, maxNs / 1000)
        }
    }
}
