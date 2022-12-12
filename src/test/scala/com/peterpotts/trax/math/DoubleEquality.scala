package com.peterpotts.trax.math

import org.scalactic.Equality
import org.scalactic.TripleEquals._

import scala.reflect.ClassTag

/**
  * @author Peter Potts
  */
object DoubleEquality {

  class ClassTagEquality[T: ClassTag](isEqualTo: (T, T) => Boolean) extends Equality[T] {
    def areEqual(left: T, other: Any): Boolean = other match {
      case right: T => isEqualTo(left, right)
      case _ => false
    }
  }

  def equality[T: ClassTag](isEqualTo: (T, T) => Boolean): Equality[T] = new ClassTagEquality(isEqualTo)

  implicit val doubleEquality: Equality[Double] =
    equality[Double] { (left, right) =>
      (left.isNaN && right.isNaN) || ((left <= right + epsilon) && (left >= right - epsilon))
    }

  implicit def equalityOption[T: Equality]: Equality[Option[T]] =
    equality[Option[T]] { (left, right) =>
      (left.isEmpty && right.isEmpty) || (left.nonEmpty && right.nonEmpty && left.get === right.get)
    }

  implicit def equalitySeq[T: Equality]: Equality[Seq[T]] =
    equality[Seq[T]] { (lefts, rights) =>
      lefts.lengthCompare(rights.size) == 0 && lefts.zip(rights).forall { case (left, right) => left === right }
    }

  implicit def equalityIndexedSeq[T: Equality]: Equality[IndexedSeq[T]] =
    equality[IndexedSeq[T]] { (lefts, rights) =>
      lefts.size == rights.size && lefts.zip(rights).forall { case (left, right) => left === right }
    }

  implicit def equalityList[T: Equality]: Equality[List[T]] =
    equality[List[T]] { (lefts, rights) =>
      lefts.lengthCompare(rights.size) == 0 && lefts.zip(rights).forall { case (left, right) => left === right }
    }

  implicit def equalitySet[T: Equality]: Equality[Set[T]] =
    equality[Set[T]] { (lefts, rights) =>
      lefts.size == rights.size && lefts.forall(left => rights.exists(right => left === right))
    }

  implicit def equalityArray[T: Equality : ClassTag]: Equality[Array[T]] =
    equality[Array[T]] { (lefts, rights) =>
      lefts.length == rights.length && lefts.indices.forall(index => lefts(index) === rights(index))
    }
}
