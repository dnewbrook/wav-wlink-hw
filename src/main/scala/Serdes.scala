package wav.wlink

import wav.common._

import chisel3._
import chisel3.util._
import chisel3.stage.ChiselStage
import chisel3.experimental.ChiselEnum


import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.ahb._
import freechips.rocketchip.amba.apb._
import freechips.rocketchip.subsystem.BaseSubsystem
import freechips.rocketchip.subsystem.CrossingWrapper
import freechips.rocketchip.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.unittest._
import freechips.rocketchip.devices.tilelink._


class WavD2DSerdesTx(val dataWidth: Int = 8, val padWidth: Int = 1)(implicit p: Parameters) extends BlackBox{
  val io = IO(new Bundle{
    val scan        = new WavScanBundle
    val clk         = Input (Bool())
    val reset       = Input (Bool())
    val clk_en      = Input (Bool())
    val ready       = Output(Bool())
    val link_data   = Input (UInt(dataWidth.W))
    val link_clk    = Output(Bool())
    val pad         = Output(Bool())
    val pad_clk     = Output(Bool())
  })
}

class D2DSerdesTxClk extends BlackBox{
  val io = IO(new Bundle{
    val clk_i = Input (Bool())
    val clk_o = Output(Bool())
  })
}


class WavD2DSerdesRx(val dataWidth: Int = 8, val padWidth: Int = 1)(implicit p: Parameters) extends BlackBox{
  val io = IO(new Bundle{
    val scan      = new WavScanBundle
    val por_reset = Input (Bool())
    
    val pol       = Input (Bool())
    val link_clk  = Output(Bool())
    val link_data = Output(UInt(dataWidth.W))
    
    val pad_clk   = Input (Bool())
    val pad       = Input (Bool())
  })  
}




class WavD2DSerdesBumpBundle(
  val numTxLanes: Int = 1,
  val numRxLanes: Int = 1
) extends Bundle{
  val clk_tx    = Output(Bool())
  val clk_tx_p    = Output(Bool())
  val tx          = Output(Vec(numTxLanes, Bool()))
  val clk_rx    = Input (Bool())
  val clk_rx_p    = Input (Bool())
  val rx          = Input (Vec(numRxLanes, Bool()))
}

class WavD2DSerdes(
  val numTxLanes: Int, 
  val numRxLanes: Int, 
  val dataWidth : Int = 16, 
  val baseAddr  : BigInt = 0x0
)(implicit p: Parameters) extends LazyModule{
  
  
  val device = new SimpleDevice("wavd2dserdes", Seq("wavious,d2dserdes"))
  val node = WavAPBRegisterNode(
    address = AddressSet.misaligned(baseAddr, 0x4),
    device  = device,
    //concurrency = 1, //make depending on apn (apb requires 1)
    beatBytes = 4,
    noRegTest = true) 
  
  lazy val module = new LazyModuleImp(this) with RequireAsyncReset{
    val io = IO(new Bundle{
      val scan        = new WavScanBundle
      val link_tx     = new WlinkPHYTxBundle(numTxLanes * dataWidth)
      val link_rx     = new WlinkPHYRxBundle(numRxLanes * dataWidth)
      val hsclk       = Input (Bool())
      val por_reset   = Input (Bool())
      val pad         = new WavD2DSerdesBumpBundle(numTxLanes, numRxLanes)
    })
    io.scan.out := false.B

    val serdestx        = Seq.tabulate(numTxLanes)(i => Module(new WavD2DSerdesTx(dataWidth)))
    val serdesrx        = Seq.tabulate(numRxLanes)(i => Module(new WavD2DSerdesRx(dataWidth)))

    val hsclk_scan    = WavClockMux(io.scan.mode, io.scan.clk, io.hsclk)
    val por_reset_scan= WavResetSync(hsclk_scan, io.por_reset, io.scan.asyncrst_ctrl)
    
    val swi_pream_count = Wire(UInt(8.W))
    val swi_post_count  = Wire(UInt(8.W))
    val swi_pol         = Wire(Bool())

    val tx_en         = Wire(Bool())

    for(i <- 0 until numTxLanes){
      serdestx(i).io.scan.connectScan(io.scan)
      serdestx(i).io.clk        := hsclk_scan
      serdestx(i).io.reset      := io.por_reset
      serdestx(i).io.clk_en     := tx_en
      serdestx(i).io.link_data  := io.link_tx.tx_link_data((dataWidth*i)+dataWidth-1, dataWidth*i)

      io.pad.tx(i)            := serdestx(i).io.pad
    }
    //Always just use lane0 for link clock
    io.link_tx.tx_link_clk    := serdestx(0).io.link_clk
    io.pad.clk_tx             := serdestx(0).io.pad_clk

    
    val precount_in      = Wire(UInt(8.W))
    val precount         = withClockAndReset(io.link_tx.tx_link_clk.asClock, por_reset_scan.asAsyncReset){RegNext(precount_in, "hf".U)}
    precount_in          := Mux(io.link_tx.tx_en, Mux(precount === 0.U, precount, precount - 1.U), swi_pream_count)
    val postcount_in     = Wire(UInt(8.W))
    val postcount        = withClockAndReset(io.link_tx.tx_link_clk.asClock, por_reset_scan.asAsyncReset){RegNext(postcount_in, 0.U)}
    postcount_in         := Mux(~io.link_tx.tx_en, Mux(postcount === 0.U, postcount, postcount - 1.U), swi_post_count)

    tx_en               := io.link_tx.tx_en | (postcount =/= 0.U && ~io.link_tx.tx_en)
    io.link_tx.tx_ready := precount === 0.U & io.link_tx.tx_en

    val rx_link_data      = Wire(Vec(numRxLanes, UInt(dataWidth.W)))
    for(i <- 0 until numRxLanes){
      serdesrx(i).io.scan.connectScan(io.scan)
      serdesrx(i).io.por_reset  := io.por_reset
      serdesrx(i).io.pol        := swi_pol
      serdesrx(i).io.pad_clk    := io.pad.clk_rx
      serdesrx(i).io.pad        := io.pad.rx(i)
      rx_link_data(i)         := serdesrx(i).io.link_data
    }
    io.link_rx.rx_link_clk    := serdesrx(0).io.link_clk
    io.link_rx.rx_link_data   := rx_link_data.asUInt
    io.link_rx.rx_data_valid  := true.B
    
    
    node.regmap(
      WavSWReg(0x0, "Control", "General Controls for serdes PHY",
        WavRW(swi_pream_count,    1.U,    "pream_count",      "Number of cycles to send the clock prior to starting data"),
        WavRW(swi_post_count,     7.U,    "post_count",       "Number of cycles to send the clock after sending data"),
        WavRW(swi_pol,            true.B, "polarity",         "Polarity of the RX sampling clock")
      ) 
    )
    
  }
}


