package com.peterpotts.example

import scalaz.Free.Trampoline
import scalaz._

import scala.concurrent.Future
import scala.util.{Failure, Success, Try}

/**
  * @author Peter Potts
  */
trait ExampleComposite {
  self: ExampleMonad with ExamplePrimitive =>

  def exampleEither[A, B](exampleA: Example[A], exampleB: Example[B]): Example[Either[A, B]] =
    for {
      condition <- exampleBoolean
      a <- exampleA
      b <- exampleB
    } yield if (condition) Left(a) else Right(b)

  def exampleOption[A](exampleA: Example[A]): Example[Option[A]] =
    for {
      condition <- exampleBoolean
      a <- exampleA
    } yield if (condition) None else Some(a)

  def exampleSome[A](exampleA: Example[A]): Example[Option[A]] = exampleA.map(Some(_))

  def exampleTry[A](exampleA: Example[A]): Example[Try[A]] =
    for {
      condition <- exampleBoolean
      a <- exampleA
    } yield if (condition) Failure(new RuntimeException) else Success(a)

  def exampleFuture[A](exampleA: Example[A]): Example[Future[A]] =
    for {
      condition <- exampleBoolean
      a <- exampleA
    } yield if (condition) Future.failed(new RuntimeException) else Future.successful(a)

  def exampleTrampoline[A](exampleA: Example[A])(implicit interpreter: Exampler ~> Id.Id): Example[Trampoline[A]] =
    Example(_ => Trampoline.delay(exampleA.foldMapRec[Id.Id](interpreter)))
}
