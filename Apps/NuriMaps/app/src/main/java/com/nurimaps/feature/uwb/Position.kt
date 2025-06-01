package com.nurimaps.feature.uwb

import com.lemmingapex.trilateration.NonLinearLeastSquaresSolver
import com.lemmingapex.trilateration.TrilaterationFunction
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer

data class Position(val x: Double, val y: Double, val z: Double)

object PositionCalculator {

    fun calculatePosition(distances: List<AnchorDistance>): Position? {
        if (distances.size < 3) return null // 최소 3개의 앵커 필요

        val positions = distances.map { anchorDistance ->
            doubleArrayOf(anchorDistance.anchor.x, anchorDistance.anchor.y, anchorDistance.anchor.z)
        }.toTypedArray()

        val distancesArray = distances.map { it.distance }.toDoubleArray()

        return try {
            val solver = NonLinearLeastSquaresSolver(
                TrilaterationFunction(positions, distancesArray),
                LevenbergMarquardtOptimizer()
            )
            val optimum = solver.solve()
            val point = optimum.point.toArray()
            Position(point[0], point[1], point[2])
        } catch (e: Exception) {
            e.printStackTrace()
            null
        }
    }
}