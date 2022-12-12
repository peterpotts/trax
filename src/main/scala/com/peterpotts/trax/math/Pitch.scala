package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
object Pitch {
  def apply(θ: Double): EulerAngles = EulerAngles(ϕ = 0, θ = θ, ψ = 0)
}
