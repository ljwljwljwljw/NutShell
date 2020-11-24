package nutcore.fu

import chisel3.{util, _}
import chisel3.util._
import utils._
import fpu._
import fpu.FPUIOFunc._
import fpu.divsqrt.DivSqrt
import fpu.fma.FMA
import nutcore._
import freechips.rocketchip.tile._
import hardfloat.{CompareRecFN, INToRecFN, RecFNToIN}

class FpInstr extends NutCoreBundle {
  val func5 = UInt(5.W)
  val fmt = UInt(2.W)
  val rs2 = UInt(5.W)
  val rs1 = UInt(5.W)
  val rm = UInt(3.W)
  val rd = UInt(5.W)
  val op = UInt(7.W)
  assert(this.getWidth == 32)
}

class FpuCsrIO extends NutCoreBundle {
  val fflags = Output(new Fflags)
  val isIllegal = Output(Bool())
  val frm = Input(UInt(3.W))
}

class FPUIO extends FunctionUnitIO{
  // use XLEN because fpu share data path with cpu
  val src3 = Input(UInt((XLEN+1).W))
  val fpu_csr = new FpuCsrIO
  val fpWen = Input(Bool())
  val instr = Input(UInt(32.W))
  val inputFunc = Input(UInt(1.W))
  val outputFunc = Input(UInt(2.W))
}

class FtoI extends FPUSubModule with HasNutCoreParameter  with HasFPUParameters {

  val op = io.in.bits.op
  val isFmv = op === FPUOpType.fmv_f2i
  val isDouble = io.in.bits.isDouble
  val tag = Mux(isDouble, D, S)
  val rm = io.in.bits.rm
  val (src1, src2) = (unbox(io.in.bits.a, tag, None), unbox(io.in.bits.b, tag, None))

  val classifyRes = Mux(isDouble,
    FType.D.classify(FType.D.unsafeConvert(src1, FType.D)),
    FType.S.classify(FType.S.unsafeConvert(src1, FType.S))
  )
  val fmvRes = ieee(src1)

  val dcmp = Module(new CompareRecFN(maxExpWidth, maxSigWidth))
  dcmp.io.a := src1
  dcmp.io.b := src2
  dcmp.io.signaling := !rm(1)

  val isCmp = op === FPUOpType.fle || op === FPUOpType.flt || op === FPUOpType.feq

  val cmpRes = ((~rm).asUInt() & Cat(dcmp.io.lt, dcmp.io.eq)).orR
  val cmpFflags = dcmp.io.exceptionFlags
  val rawA = hardfloat.rawFloatFromRecFN(maxExpWidth, maxSigWidth, src1)
  Debug(flag = false, cond = io.in.fire()){
    printf(p"src1: ${Hexadecimal(src1)} src2:${Hexadecimal(src2)} eq:${dcmp.io.eq} lt:${dcmp.io.lt} gt:${dcmp.io.gt} rm:${rm}\n")
    printf(p"nan: ${rawA.isNaN} inf:${rawA.isInf}" +
      p" ieee(unbox):${Hexadecimal(ieee(src1))} ieee:${Hexadecimal(ieee(io.in.bits.a))}\n")
  }




  val signedOut = op === FPUOpType.f2l || op === FPUOpType.f2w
  val toLong = op === FPUOpType.f2l || op === FPUOpType.f2lu
  val toWord = op === FPUOpType.f2w || op === FPUOpType.f2wu

  val ftoW = Module(new RecFNToIN(maxExpWidth, maxSigWidth, 32))
  val ftoL = Module(new RecFNToIN(maxExpWidth, maxSigWidth, 64))

  for(m <- Seq(ftoW, ftoL)){
    m.io.in := src1
    m.io.roundingMode := rm
    m.io.signedOut := signedOut
  }

  val cvtRes = Mux(toLong, ftoL.io.out, ftoW.io.out)
  val cvtIntFlags = Mux(toLong, ftoL.io.intExceptionFlags, ftoW.io.intExceptionFlags)
  val cvtFflags = Cat(cvtIntFlags(2, 1).orR(), 0.U(3.W), cvtIntFlags(0))

  io.out.bits.result := MuxCase(classifyRes, Seq(
    isFmv -> fmvRes,
    (toLong || toWord) -> cvtRes,
    isCmp -> cmpRes
  ))
  io.out.bits.fflags := MuxCase(0.U, Seq(
    (toLong || toWord) -> cvtFflags,
    isCmp -> cmpFflags
  )).asTypeOf(new Fflags)
  io.out.valid := io.in.valid
  io.in.ready := io.out.ready
}

class ItoF extends FPUSubModule with HasNutCoreParameter with HasFPUParameters {

  val intSrc = io.in.bits.a(XLEN-1, 0)
  val op = io.in.bits.op
  val isFmv = op === FPUOpType.fmv_i2f
  val tag = Mux(io.in.bits.isDouble, D, S)
  val rm = io.in.bits.rm
  val isDouble = io.in.bits.isDouble

  val fmvRes = Mux(io.in.bits.isDouble, recode(intSrc, D), recode(intSrc(31, 0), S))

  val intValue = MuxLookup(op, intSrc, Seq(
    FPUOpType.w2f -> SignExt(intSrc(31, 0), XLEN),
    FPUOpType.wu2f -> ZeroExt(intSrc(31, 0), XLEN)
  ))
  val signedInt = op === FPUOpType.w2f || op === FPUOpType.l2f

  val intToD = Module(new INToRecFN(XLEN, FType.D.exp, FType.D.sig))
  val intToS = Module(new INToRecFN(XLEN, FType.S.exp, FType.S.sig))

  for(m <- Seq(intToD, intToS)){
    m.io.signedIn := signedInt
    m.io.in := intValue
    m.io.roundingMode := rm
    m.io.detectTininess := hardfloat.consts.tininess_afterRounding
  }
  val fcvtRes = Mux(isDouble,
    sanitizeNaN(intToD.io.out, FType.D),
    sanitizeNaN(intToS.io.out, FType.S)
  )
  val fcvtFflags = Mux(isDouble, intToD.io.exceptionFlags, intToS.io.exceptionFlags)

  io.out.bits.fflags := Mux(isFmv, 0.U.asTypeOf(new Fflags), fcvtFflags.asTypeOf(new Fflags))
  io.out.bits.result := Mux(isFmv, fmvRes, fcvtRes)
  io.out.valid := io.in.valid
  io.in.ready := io.out.ready
}


class FtoF extends FPUSubModule with HasNutCoreParameter with HasFPUParameters {

  val op = io.in.bits.op
  val isDouble = io.in.bits.isDouble
  val tag = Mux(isDouble, D, S)
  val rm = io.in.bits.rm
  val (src1, src2) = (unbox(io.in.bits.a, tag, None), unbox(io.in.bits.b, tag, None))

  val dcmp = Module(new CompareRecFN(maxExpWidth, maxSigWidth))
  dcmp.io.a := src1
  dcmp.io.b := src2
  dcmp.io.signaling := !rm(1)

  val lt = dcmp.io.lt
  val isFminmax = op === FPUOpType.fmin || op === FPUOpType.fmax
  val isFsgnj = op === FPUOpType.fsgnj || op === FPUOpType.fsgnjn || op === FPUOpType.fsgnjx

  val signNum = Mux(rm(1), src1 ^ src2, Mux(rm(0), ~src2, src2))
  val fsgnjRes = Cat(signNum(fLen), src1(fLen-1, 0))
  val fsgnjMux = WireInit(fsgnjRes)

  val isnan1 = maxType.isNaN(src1)
  val isnan2 = maxType.isNaN(src2)
  val isInvalid = maxType.isSNaN(src1) || maxType.isSNaN(src2)
  val isNaNOut = isnan1 && isnan2
  val isLHS = isnan2 || rm(0) =/= lt && !isnan1
  val fminFflags = isInvalid << 4
  val fminResults = Mux(isLHS, src1, src2)

  when(isFminmax){
    fsgnjMux := fminResults
  }

  val mux = WireInit(fsgnjMux)
  when(!isDouble){
    mux := Cat(fsgnjMux)
  }







}




class FPU extends NutCoreModule{
  //  require(XLEN >= FLEN)
  val io = IO(new FPUIO)
  val (valid, src1, src2, src3, func) =
    (io.in.valid, io.in.bits.src1, io.in.bits.src2, io.src3, io.in.bits.func)

  def access(valid: Bool, src1: UInt, src2: UInt, src3: UInt, func: UInt): UInt = {
    this.valid := valid
    this.src1 := src1
    this.src2 := src2
    this.src3 := src3
    this.func := func
    io.out.bits
  }

  val instr = io.instr.asTypeOf(new FpInstr)
  val isRVD = instr.fmt(0)
  val src = VecInit(Seq(src1, src2, src3))

  val roudingMode = Mux(instr.rm===7.U, io.fpu_csr.frm, instr.rm)
  val op = func

  val subModuleInput = Wire(new FPUSubModuleInput)
  subModuleInput.a := src(0)
  subModuleInput.b := src(1)
  subModuleInput.c := src(2)
  subModuleInput.op := op
  subModuleInput.isDouble := isRVD
  subModuleInput.rm := roudingMode

  val ftoIUnit = Module(new FtoI)
  val itoFUnit = Module(new ItoF)


  def isOneOf(key: UInt, set: Seq[UInt]):Bool = {
    Cat(set.map(_ === key)).orR()
  }

  val isFtoI = isOneOf(
    op,
    Seq(
      FPUOpType.fclass,
      FPUOpType.fmv_f2i,
      FPUOpType.f2w,
      FPUOpType.f2wu,
      FPUOpType.f2l,
      FPUOpType.f2lu,
      FPUOpType.fle,
      FPUOpType.feq,
      FPUOpType.flt
    )
  )
  val isItoF = isOneOf(
    op,
    Seq(
      FPUOpType.fmv_i2f,
      FPUOpType.l2f,
      FPUOpType.lu2f,
      FPUOpType.w2f,
      FPUOpType.wu2f
    )
  )

  ftoIUnit.io.in.bits := subModuleInput
  ftoIUnit.io.in.valid := io.in.valid && isFtoI
  ftoIUnit.io.out.ready := true.B
  itoFUnit.io.in.bits := subModuleInput
  itoFUnit.io.in.valid := io.in.valid && isItoF
  itoFUnit.io.out.ready := true.B

  val output = Mux(isItoF,
    itoFUnit.io.out.bits,
    ftoIUnit.io.out.bits
  )


  def printSubModule(m: FPUSubModule): Unit ={
    Debug(flag = true, cond = m.io.in.fire()){
      printf(p"isDouble:${Hexadecimal(isRVD)} in0: ${Hexadecimal(src1)} in1:${Hexadecimal(src2)} out:${m.io.out.bits.result} v:${m.io.out.fire()}\n")
    }
  }
  printSubModule(ftoIUnit)
  printSubModule(itoFUnit)


  io.in.ready := io.out.ready
  io.out.valid := io.in.valid
  io.out.bits := output.result

  //TODO: check illegal rounding mode exception
  io.fpu_csr.isIllegal := false.B
  io.fpu_csr.fflags := Mux(io.out.valid, output.fflags, 0.U.asTypeOf(new Fflags))
}
