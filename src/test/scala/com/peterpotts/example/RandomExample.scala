package com.peterpotts.example

import java.security.SecureRandom

import scala.util.Random

/**
  * @author Peter Potts
  */
object RandomExample extends RandomExample(Random)

object SecureRandomExample extends RandomExample(new SecureRandom)

class RandomExample(random: Random) extends Example {
  private[example] def nextLong() = random.nextLong()

  val examplePerson: Example[Person] =
    for {
      id <- exampleUUID
      name <- exampleString
      age <- exampleInt
      email <- exampleOption(exampleString)
      friends <- exampleList(exampleString)
    } yield Person(id, name, age, email, friends)
}
