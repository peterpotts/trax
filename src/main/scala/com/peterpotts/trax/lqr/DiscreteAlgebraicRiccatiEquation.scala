package com.peterpotts.trax.lqr

import com.peterpotts.trax.math.MatrixDDecorator._
import com.peterpotts.trax.math._

import scala.annotation.tailrec

/**
  * @author Peter Potts
  */
case class DiscreteAlgebraicRiccatiEquation(
  nx: Int,
  nu: Int,
  A: MatrixD,
  B: MatrixD,
  Q: MatrixD,
  R: MatrixD
) {
  require(A.is(nx, nx), s"A must be $nx by $nx matrix")
  require(B.is(nx, nu), s"B must be $nx by $nu matrix")
  require(Q.is(nx, nx), s"Q must be $nx by $nx matrix")
  require(R.is(nu, nu), s"R must be $nu by $nu matrix")

  private def iteration(P: MatrixD): MatrixD = {
    val ATP = A.transpose *** P
    val BTP = B.transpose *** P
    ATP *** A - ATP *** B *** (R + BTP *** B).inverse *** B.transpose *** P *** A + Q
  }

  def iterate(P: MatrixD): MatrixD = {
    @tailrec def loop(level: Int, P: MatrixD): MatrixD = {
      val nextP = iteration(P)

      if (P === nextP) {
        P
      } else if (level == 1000) {
        println("Iteration overflow")
        nextP
      } else {
        loop(level + 1, nextP)
      }
    }

    loop(0, P)
  }

  def gain(P: MatrixD): MatrixD = {
    val BTP = B.transpose *** P
    (R + BTP *** B).inverse *** BTP *** A
  }

  lazy val P: MatrixD = iterate(Q)
  lazy val K: MatrixD = gain(P)

  def isSolution(P: MatrixD): Boolean = P == iteration(P)
}
