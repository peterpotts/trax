package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
object Yaw {
  def apply(ψ: Double): EulerAngles = EulerAngles(ϕ = 0, θ = 0, ψ = ψ)
}
