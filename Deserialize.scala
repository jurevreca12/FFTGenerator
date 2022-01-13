//******************************************************************************
// Copyright (c) 2021-2022, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE for license details.
//------------------------------------------------------------------------------
// Author: James Dunn, UC Berkeley (dunn@eecs.berkeley.edu)

package fftgenerator

import chisel3._
import chisel3.experimental.FixedPoint
import chisel3.util.Decoupled
import chisel3.util._
import dspjunctions._
import dsptools.numbers._
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.subsystem.BaseSubsystem
import scala.math.sqrt

/**
  * This block performs a deserialization. It takes time-series inputs on a
  * decoupled interface. These inputs are collected into a vector of registers.
  * This vector of registers is mapped to an output vector IO. When a counter
  * reaches the deserialization factor, output.valid is asserted.
  */

/**
	* Base class for Deserialize parameters.
	* These are type-generic.
	*/

trait DeserializeParams[T <: Data] {
	// Datatype of input and output samples
	val protoInDes, protoOutDes: DspComplex[T]
	// Deserialization factor
	val lanes: Int
  // Number of subcarriers
  val S: Int
}

/**
	* Deserialize parameters for fixed-point implementation.
	*/
case class FixedDeserializeParams(
  IOWidth: Int,
  binaryPoint: Int,
  lanes: Int,
  S: Int,
) extends DeserializeParams[FixedPoint] {
	  val protoInDes = DspComplex(FixedPoint(IOWidth.W, (binaryPoint-3).BP),FixedPoint(IOWidth.W, (binaryPoint-3).BP))
    val protoOutDes = DspComplex(FixedPoint(IOWidth.W, binaryPoint.BP),FixedPoint(IOWidth.W, binaryPoint.BP))
}

/**
  * Bundle type as IO for Deserialize modules
  */
class DeserializeIO[T <: Data](params: DeserializeParams[T]) extends Bundle {
  // Input samples (single input, time-series)

  val in = Flipped(Decoupled(params.protoInDes.cloneType))

  // Deserialized output (parallel vector)
  val out = Output(ValidWithSync(Vec(params.lanes, params.protoOutDes.cloneType)))

  override def cloneType: this.type = DeserializeIO(params).asInstanceOf[this.type]
}
object DeserializeIO {
  def apply[T <: Data](params: DeserializeParams[T]): DeserializeIO[T] =
    new DeserializeIO(params)
}

/**
	* Here is the Deserializer itself.
	*/
  // todo ask abe/yue: why are protoIn and protoOut different? Arbitrarily shifts bits
class Deserialize[T <: Data : Real](val params: DeserializeParams[T]) extends Module {

  val io = IO(DeserializeIO(params))

  // Temporarily leave sync false (it's optional for FFT)
  io.out.sync := false.B

  // Create vec of registers to hold deserialized values
  val serializedBits = Reg(Vec(params.lanes, params.protoOutDes.cloneType))

  // Wire up deserialization registers to outputs.
  io.out.bits := serializedBits

  // nCounter keeps track of how far along in the serialization we are
  val nCounter = RegInit(0.U((log2Ceil(params.lanes) + 1).W))

  // THIS IS A HACK TO FIX SCALING
  val scale = Wire(params.protoOutDes.cloneType)
  scale.real := ConvertableTo[T].fromDouble(1.0/params.S)
  scale.imag := ConvertableTo[T].fromDouble(0.0)

  // Enumerate FSM states
  // val INIT = 0.U(3.W)
  val COUNTING = 2.U(3.W)
  // val SUFFIX = 3.U(3.W)
  val DONE = 4.U(3.W)

  // State machine
  val state = RegInit(COUNTING)

  io.in.ready := state === COUNTING
  io.out.valid := state === DONE

  // TODO: HOW DO WE RESET THE FSM?
  switch (state) {
    is (COUNTING) {
      when(nCounter === params.lanes.U) {
        state := DONE
      }.otherwise {
        state := COUNTING
        when (io.in.valid) {
          serializedBits(nCounter) := io.in.bits
          nCounter := nCounter + 1.U
        }
      }
    }

    is (DONE) {
      // reset for next use
      nCounter := 0.U
      state := COUNTING
    }
  }

  // when (state === INIT) {
  //   io.in.ready := true.B
  //   io.out.valid := false.B
  //   when (io.in.valid) {
  //     state := COUNTING
  //   }.otherwise {
  //     state := INIT
  //   }
  // }.elsewhen (state === COUNTING) {
  //   io.in.ready := true.B
  //   io.out.valid := false.B
  //     when (io.in.valid) {
  //       when (nCounter < (params.lanes - 1.U)) {
  //         serializedBits(nCounter) := io.in.bits * scale
  //         nCounter := nCounter + 1.U
  //         state := COUNTING
  //       }.otherwise {
  //         // TODO ASK ABE: KINDA HACKY... IS THIS A SYNCHRONOUS BLOCK? how do i formulate the fsm here
  //         // THIS IS A HACK TO FIX SCALING
  //         serializedBits(nCounter) := io.in.bits * scale
  //         state := SUFFIX
  //         nCounter := 0.U
  //       }
  //     }.elsewhen(nCounter === (params.lanes - 1.U)) {
  //       state := SUFFIX
  //     }.otherwise {
  //       state := COUNTING
  //     }
  // }.elsewhen (state === SUFFIX) {
  //   io.in.ready := true.B
  //   io.out.valid := true.B
  //   state := DONE
  // }.elsewhen (state === DONE) { // todo: reset state
  //   io.in.ready := false.B
  //   io.out.valid := false.B
  //   state := DONE
  // }.otherwise { // Bad state
  //   io.in.ready := false.B
  //   io.out.valid := false.B
  //   state := INIT
  //   nCounter := 0.U
  // }
}
