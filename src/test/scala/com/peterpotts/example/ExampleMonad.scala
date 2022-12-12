package com.peterpotts.example

import scalaz._

/**
  * @author Peter Potts
  */
trait ExampleMonad {

  case class Exampler[+T](generator: Long => T)

  type Example[T] = Free[Exampler, T]

  implicit val examplerFunctor: Functor[Exampler] = new Functor[Exampler] {
    def map[A, B](exampler: Exampler[A])(f: A => B): Exampler[B] = Exampler(seed => f(exampler.generator(seed)))
  }

  object Example {
    def apply[T](generator: Long => T): Example[T] = Free.liftF(Exampler(generator))
  }

}