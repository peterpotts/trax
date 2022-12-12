package com.peterpotts.example

import scalaz.Scalaz._

/**
  * @author Peter Potts
  */
trait ExamplePrimitive {
  self: ExampleMonad =>

  val defaultSize = 4

  val exampleLong: Example[Long] = Example[Long](identity)

  val exampleInt: Example[Int] = Example(_.toInt)

  def exampleInt(until: Int): Example[Int] = Example(long => math.abs(long.toInt % until))

  val exampleBoolean: Example[Boolean] = Example(_ > 0)

  val exampleByte: Example[Byte] = Example(_.toByte)

  val exampleDouble: Example[Double] = {
    val maximum = math.max(-Long.MinValue.toDouble, Long.MaxValue.toDouble)
    Example(long => long.toDouble / maximum)
  }

  val exampleUnit: Example[Unit] = Example(_ => ())

  def exampleLift[A](a: A): Example[A] = a.point[Example]
}
