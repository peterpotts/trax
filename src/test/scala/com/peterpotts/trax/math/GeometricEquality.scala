package com.peterpotts.trax.math

import com.peterpotts.trax.math.DoubleEquality._
import org.scalactic.Equality
import org.scalactic.TripleEquals._

/**
  * @author Peter Potts
  */
object GeometricEquality {
  implicit val vector1Equality: Equality[Vector1] =
    equality[Vector1] { (left, right) =>
      left === right
    }

  implicit val vector2Equality: Equality[Vector2] =
    equality[Vector2] { (left, right) =>
      left === right
    }

  implicit val vector3Equality: Equality[Vector3] =
    equality[Vector3] { (left, right) =>
      left === right
    }

  implicit val vector4Equality: Equality[Vector4] =
    equality[Vector4] { (left, right) =>
      left === right
    }

  implicit val matrix12Equality: Equality[Matrix12] =
    equality[Matrix12] { (left, right) =>
      left === right
    }

  implicit val matrix22Equality: Equality[Matrix22] =
    equality[Matrix22] { (left, right) =>
      left === right
    }

  implicit val matrix33Equality: Equality[Matrix33] =
    equality[Matrix33] { (left, right) =>
      left === right
    }

  implicit val matrix34Equality: Equality[Matrix34] =
    equality[Matrix34] { (left, right) =>
      left === right
    }

  implicit val matrix43Equality: Equality[Matrix43] =
    equality[Matrix43] { (left, right) =>
      left === right
    }

  implicit val matrix44Equality: Equality[Matrix44] =
    equality[Matrix44] { (left, right) =>
      left === right
    }

  implicit val tensor333Equality: Equality[Tensor333] =
    equality[Tensor333] { (left, right) =>
      left === right
    }

  implicit val tensor334Equality: Equality[Tensor334] =
    equality[Tensor334] { (left, right) =>
      left === right
    }

  implicit val tensor343Equality: Equality[Tensor343] =
    equality[Tensor343] { (left, right) =>
      left === right
    }

  implicit val tensor433Equality: Equality[Tensor433] =
    equality[Tensor433] { (left, right) =>
      left === right
    }

  implicit val eulerAnglesEquality: Equality[EulerAngles] =
    equality[EulerAngles] { (left, right) =>
      left === right
    }

  implicit val quaternionEquality: Equality[Quaternion] =
    equality[Quaternion] { (left, right) =>
      left === right
    }

  implicit val unitQuaternionEquality: Equality[UnitQuaternion] =
    equality[UnitQuaternion] { (left, right) =>
      left === right
    }

  implicit val rotationMatrixEquality: Equality[RotationMatrix33] =
    equality[RotationMatrix33] { (left, right) =>
      left.matrix33 === right.matrix33
    }

  implicit val axisAngleEquality: Equality[AxisAngle] =
    equality[AxisAngle] { (left, right) =>
      left === right
    }

  implicit val rotationVectorEquality: Equality[RotationVector] =
    equality[RotationVector] { (left, right) =>
      left === right
    }

  implicit val vectorDEquality: Equality[VectorD] =
    equality[VectorD] { (left, right) =>
      left.zip(right).forall {
        case (x, y) => x === y
      }
    }

  implicit val matrixDEquality: Equality[MatrixD] =
    equality[MatrixD] { (left, right) =>
      left.zip(right).forall {
        case (x, y) => x === y
      }
    }

}
