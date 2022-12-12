package com.peterpotts.example

import scalaz._

/**
  * @author Peter Potts
  */
trait Example extends
  ExampleMonad with
  ExamplePrimitive with
  ExampleComposite with
  ExampleTuple with
  ExampleCollection with
  ExampleSpecial {
  private[example] def nextLong(): Long

  implicit object Interpreter extends (Exampler ~> Id.Id) {
    def apply[A](exampler: Exampler[A]): Id.Id[A] = exampler.generator(nextLong())
  }

  implicit class DecoratedExample[T](example: Example[T]) {
    def next(): T = example.foldMapRec[Id.Id](Interpreter)
  }

}
