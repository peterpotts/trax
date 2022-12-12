package com.peterpotts.trax.math

/**
  * @author Peter Potts
  */
object Roll {
  def apply(ϕ: Double): EulerAngles = EulerAngles(ϕ = ϕ, θ = 0, ψ = 0)
}
