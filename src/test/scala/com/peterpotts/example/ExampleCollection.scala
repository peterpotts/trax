package com.peterpotts.example

import scalaz.Scalaz._
import scalaz._

import scala.collection.immutable.IndexedSeq
import scala.reflect.ClassTag

/**
  * @author Peter Potts
  */
trait ExampleCollection {
  self: ExampleMonad with ExamplePrimitive with ExampleTuple =>

  def examplePick[A](values: IndexedSeq[A]): Example[A] = exampleInt(values.size).map(values(_))

  def exampleShuffle[A](examples: Example[A]*): Example[A] = exampleInt(examples.size).flatMap(examples(_))

  def exampleStream[A](exampleA: Example[A])(implicit interpreter: Exampler ~> Id.Id): Example[Stream[A]] =
    Example(_ => Stream.continually(exampleA.foldMapRec[Id.Id](interpreter)))

  def exampleList[A](exampleA: Example[A], size: Int = defaultSize): Example[List[A]] =
    List.fill(size)(exampleA).sequence[Example, A]

  def exampleList[A](exampleA: Example[A], sizes: IndexedSeq[Int]): Example[List[A]] =
    examplePick(sizes).flatMap(size => exampleList(exampleA, size))

  def exampleSet[A](exampleA: Example[A], size: Int = defaultSize): Example[Set[A]] =
    exampleList(exampleA, size).map(_.toSet[A])

  def exampleSet[A](exampleA: Example[A], sizes: IndexedSeq[Int]): Example[Set[A]] =
    examplePick(sizes).flatMap(size => exampleSet(exampleA, size))

  def exampleArray[A: ClassTag](exampleA: Example[A], size: Int = defaultSize): Example[Array[A]] =
    exampleList(exampleA, size).map(_.toArray[A])

  def exampleArray[A: ClassTag](exampleA: Example[A], sizes: IndexedSeq[Int]): Example[Array[A]] =
    examplePick(sizes).flatMap(size => exampleArray(exampleA, size))

  def exampleVector[A](exampleA: Example[A], size: Int = defaultSize): Example[Vector[A]] =
    exampleList(exampleA, size).map(_.toVector)

  def exampleVector[A: ClassTag](exampleA: Example[A], sizes: IndexedSeq[Int]): Example[Vector[A]] =
    examplePick(sizes).flatMap(size => exampleVector(exampleA, size))

  def exampleMap[A, B](exampleA: Example[A], exampleB: Example[B], size: Int = defaultSize): Example[Map[A, B]] =
    exampleList(exampleTuple2(exampleA, exampleB), size).map(_.toMap)

  def exampleMap[A, B](exampleA: Example[A], exampleB: Example[B], sizes: IndexedSeq[Int]): Example[Map[A, B]] =
    examplePick(sizes).flatMap(size => exampleMap(exampleA, exampleB, size))

  val exampleBytes: Example[Array[Byte]] = exampleArray(exampleByte, defaultSize)

  def exampleBytes(size: Int): Example[Array[Byte]] = exampleArray(exampleByte, size)

  def exampleBytes(sizes: IndexedSeq[Int]): Example[Array[Byte]] = examplePick(sizes).flatMap(exampleBytes(_))
}
