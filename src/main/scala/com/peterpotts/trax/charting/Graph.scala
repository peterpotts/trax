package com.peterpotts.trax.charting

import scalafx.application.JFXApp
import scalafx.application.JFXApp.PrimaryStage
import scalafx.collections.ObservableBuffer
import scalafx.scene.Scene
import scalafx.scene.chart.{LineChart, NumberAxis, XYChart}

/**
  * @author Peter Potts
  */
case class Graph(
  chartTitle: String,
  xAxis: String,
  yAxis: String
) extends LineChart(NumberAxis(xAxis), NumberAxis(yAxis)) {
  title = chartTitle

  delegate.getXAxis.setAutoRanging(true)
  delegate.getYAxis.setAutoRanging(true)

  def addSeries(
    seriesName: String,
    seriesData: Seq[(Double, Double)]
  ): Unit = {
    val items = seriesData.map {
      case (x, y) => XYChart.Data[Number, Number](x, y)
    }

    val data = ObservableBuffer(items)
    val series = XYChart.Series[Number, Number](seriesName, data)
    delegate.getData.add(series)
  }
}

object Graph extends JFXApp {
  val lineChart = Graph(
    chartTitle = "Chart title",
    xAxis = "X axis label",
    yAxis = "Y axis label"
  )

  lineChart.addSeries(
    seriesName = "Series name",
    seriesData = Seq(
      (1, 23),
      (2, 14),
      (3, 15),
      (4, 24),
      (5, 34),
      (6, 36),
      (7, 22),
      (8, 45),
      (9, 43),
      (10, 17),
      (11, 29),
      (12, 25)
    )
  )

  stage = new PrimaryStage {
    title = "Line Chart Sample"
    scene = new Scene(800, 600) {
      root = lineChart
    }
  }
}
