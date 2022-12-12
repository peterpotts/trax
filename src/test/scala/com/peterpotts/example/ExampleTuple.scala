package com.peterpotts.example

/**
  * @author Peter Potts
  */
trait ExampleTuple {
  self: ExampleMonad =>

  def exampleTuple2[A, B](exampleA: Example[A], exampleB: Example[B]): Example[(A, B)] =
    for {
      a <- exampleA
      b <- exampleB
    } yield Tuple2(a, b)

  def exampleTuple3[A, B, C](exampleA: Example[A], exampleB: Example[B], exampleC: Example[C]): Example[(A, B, C)] =
    for {
      a <- exampleA
      b <- exampleB
      c <- exampleC
    } yield Tuple3(a, b, c)

  def exampleTuple4[A, B, C, D](
    exampleA: Example[A],
    exampleB: Example[B],
    exampleC: Example[C],
    exampleD: Example[D]): Example[(A, B, C, D)] =
    for {
      a <- exampleA
      b <- exampleB
      c <- exampleC
      d <- exampleD
    } yield Tuple4(a, b, c, d)

  def exampleTuple5[A, B, C, D, E](
    exampleA: Example[A],
    exampleB: Example[B],
    exampleC: Example[C],
    exampleD: Example[D],
    exampleE: Example[E]): Example[(A, B, C, D, E)] =
    for {
      a <- exampleA
      b <- exampleB
      c <- exampleC
      d <- exampleD
      e <- exampleE
    } yield Tuple5(a, b, c, d, e)

  def exampleTuple6[A, B, C, D, E, F](
    exampleA: Example[A],
    exampleB: Example[B],
    exampleC: Example[C],
    exampleD: Example[D],
    exampleE: Example[E],
    exampleF: Example[F]): Example[(A, B, C, D, E, F)] =
    for {
      a <- exampleA
      b <- exampleB
      c <- exampleC
      d <- exampleD
      e <- exampleE
      f <- exampleF
    } yield Tuple6(a, b, c, d, e, f)

  def exampleTuple5[A, B, C, D, E, F, G](
    exampleA: Example[A],
    exampleB: Example[B],
    exampleC: Example[C],
    exampleD: Example[D],
    exampleE: Example[E],
    exampleF: Example[F],
    exampleG: Example[G]): Example[(A, B, C, D, E, F, G)] =
    for {
      a <- exampleA
      b <- exampleB
      c <- exampleC
      d <- exampleD
      e <- exampleE
      f <- exampleF
      g <- exampleG
    } yield Tuple7(a, b, c, d, e, f, g)

  def exampleTuple8[A, B, C, D, E, F, G, H](
    exampleA: Example[A],
    exampleB: Example[B],
    exampleC: Example[C],
    exampleD: Example[D],
    exampleE: Example[E],
    exampleF: Example[F],
    exampleG: Example[G],
    exampleH: Example[H]): Example[(A, B, C, D, E, F, G, H)] =
    for {
      a <- exampleA
      b <- exampleB
      c <- exampleC
      d <- exampleD
      e <- exampleE
      f <- exampleF
      g <- exampleG
      h <- exampleH
    } yield Tuple8(a, b, c, d, e, f, g, h)
}
