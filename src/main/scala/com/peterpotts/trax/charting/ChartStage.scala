package com.peterpotts.trax.charting

import com.peterpotts.trax.math._
import com.peterpotts.trax.models.Model
import scalafx.application.JFXApp.PrimaryStage
import scalafx.scene.Scene
import scalafx.scene.control._

/**
  * @author Peter Potts
  */
case class ChartStage(model: Model) extends PrimaryStage {
  title = model.title

  private val dTabs: IndexedSeq[Tab] = model.stateLabels.indices.flatMap { index =>
    def unitQuaternion(offset: Int): VectorD => Double = { vectorD =>
      val uq = Quaternion(Vector4(
        vectorD(index),
        vectorD(index + 1),
        vectorD(index + 2),
        vectorD(index + 3)
      )).versor

      val ea = uq.eulerAngles
      ea.vector3.vectorD(offset)
    }

    def rotationVector(offset: Int): VectorD => Double = { vectorD =>
      val r = RotationVector(Vector3(
        vectorD(index),
        vectorD(index + 1),
        vectorD(index + 2)
      ))

      val ea = r.unitQuaternion.eulerAngles
      ea.vector3.vectorD(offset)
    }

    val dTab =
      model.stateLabels(index) match {
        case "q.w" =>
          IndexedSeq(
            "roll" -> unitQuaternion(0),
            "pitch" -> unitQuaternion(1),
            "yaw" -> unitQuaternion(2)
          )
        case "r.x" =>
          IndexedSeq(
            "roll" -> rotationVector(0),
            "pitch" -> rotationVector(1),
            "yaw" -> rotationVector(2)
          )
        case _ => IndexedSeq.empty
      }

    dTab.map {
      case (label, get) =>
        val lineChart = Graph(
          chartTitle = label,
          xAxis = "Time",
          yAxis = label
        )

        lineChart.addSeries(
          seriesName = "Actual",
          seriesData = model.steps.map { step =>
            step.time -> get(step.actual.vectorD)
          }
        )

        lineChart.addSeries(
          seriesName = "Target",
          seriesData = model.steps.map { step =>
            step.time -> get(step.target.vectorD)
          }
        )

        lineChart.addSeries(
          seriesName = "Estimate",
          seriesData = model.steps.map { step =>
            step.time -> get(step.estimate.x.vectorD)
          }
        )

        new Tab {
          text = label
          content = lineChart
          closable = false
        }
    }
  }

  private val xTabs = model.stateLabels.indices.map { index =>
    val label = model.stateLabels(index)
    val factor = if (label == "theta.z") 1.0.toDegrees else 1.0

    val lineChart = Graph(
      chartTitle = label,
      xAxis = "Time",
      yAxis = label
    )

    lineChart.addSeries(
      seriesName = "Actual",
      seriesData = model.steps.map { step =>
        step.time -> step.actual.vectorD(index) * factor
      }
    )

    lineChart.addSeries(
      seriesName = "Target",
      seriesData = model.steps.map { step =>
        step.time -> step.target.vectorD(index) * factor
      }
    )

    lineChart.addSeries(
      seriesName = "Estimate",
      seriesData = model.steps.map { step =>
        step.time -> step.estimate.x.vectorD(index) * factor
      }
    )

    new Tab {
      text = label
      content = lineChart
      closable = false
    }
  }

  private val uTabs = model.controlLabels.indices.map { index =>
    val label = model.controlLabels(index)

    val lineChart = Graph(
      chartTitle = label,
      xAxis = "Time",
      yAxis = label
    )

    lineChart.addSeries(
      seriesName = "Control",
      seriesData = model.steps.map { step =>
        step.time -> step.u.vectorD(index)
      }
    )

    new Tab {
      text = label
      content = lineChart
      closable = false
    }
  }

  private val zTabs = model.measureLabels.indices.map { index =>
    val label = model.measureLabels(index)

    val lineChart = Graph(
      chartTitle = label,
      xAxis = "Time",
      yAxis = label
    )

    lineChart.addSeries(
      seriesName = "Measure",
      seriesData = model.steps.map { step =>
        step.time -> step.z.vectorD(index)
      }
    )

    new Tab {
      text = label
      content = lineChart
      closable = false
    }
  }

  private val lineChartTabs = dTabs ++ xTabs ++ uTabs ++ zTabs

  scene = new Scene(1400, 800) {
    root = new TabPane {
      tabs = lineChartTabs
    }

    stylesheets = List(getClass.getResource("/styles.css").toExternalForm)
  }
}
