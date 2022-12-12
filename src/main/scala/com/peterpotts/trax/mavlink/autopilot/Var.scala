package com.peterpotts.trax.mavlink.autopilot

import com.peterpotts.trax.common.Logging

import java.util.concurrent.atomic.AtomicBoolean

/**
  * @author Peter Potts
  */
class Var[T](observers: Int) extends Logging {
  private val flags = IndexedSeq.fill(observers)(new AtomicBoolean(false))

  protected var maybeLastMessage: Option[T] = None

  def set(message: T): Unit = {
    maybeLastMessage = Some(message)
    flags.foreach(_.set(true))
  }

  def get(index: Int): Option[T] =
    if (index == -1 || flags(index).getAndSet(false)) maybeLastMessage else None
}
