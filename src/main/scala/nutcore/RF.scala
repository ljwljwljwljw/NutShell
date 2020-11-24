/**************************************************************************************
* Copyright (c) 2020 Institute of Computing Technology, CAS
* Copyright (c) 2020 University of Chinese Academy of Sciences
* 
* NutShell is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2. 
* You may obtain a copy of Mulan PSL v2 at:
*             http://license.coscl.org.cn/MulanPSL2 
* 
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR 
* FIT FOR A PARTICULAR PURPOSE.  
*
* See the Mulan PSL v2 for more details.  
***************************************************************************************/

package nutcore

import chisel3._
import chisel3.util._
import chisel3.util.experimental.BoringUtils

import utils._

trait HasRegFileParameter {
  val NRReg = 32
}

class RegFile(hasZero: Boolean = true, len: Int = 64) extends HasRegFileParameter with HasNutCoreParameter {
  val rf = Mem(NRReg, UInt(len.W))
  def read(addr: UInt) : UInt = if(hasZero) Mux(addr === 0.U, 0.U, rf(addr)) else rf(addr)
  def write(addr: UInt, data: UInt) = { rf(addr) := data(len-1,0) }
} 

class ScoreBoard(hasZero: Boolean = true) extends HasRegFileParameter {
  val busy = RegInit(0.U(NRReg.W))
  def isBusy(idx: UInt): Bool = busy(idx)
  def mask(idx: UInt) = (1.U(NRReg.W) << idx)(NRReg-1, 0)
  def update(setMask: UInt, clearMask: UInt) = {
    // When clearMask(i) and setMask(i) are both set, setMask(i) wins.
    // This can correctly record the busy bit when reg(i) is written
    // and issued at the same cycle.
    // Note that rf(0) is always free.
    if(hasZero) busy := Cat(((busy & ~clearMask) | setMask)(NRReg-1, 1), 0.U(1.W))
    else busy := ((busy & ~clearMask) | setMask)
  }
}
