// ============================================================================
//  tb_cnn_accel.v  -  Testbench for CNN_TOP  (Vivado xsim compatible)
//
//  Config: 64x64 image, dilation=2
//    OUT_W = 60  →  3600 output pixels
//
//  Bug fixes vs previous version:
//   1. Load-input timeout 12000 → 20000  (each pixel needs 4 cycles,
//      4096*4 = 16384 cycles minimum).
//   2. $dumpfile/$dumpvars removed  (not supported in Vivado xsim).
//   3. verify_input_buffer waits 2 extra cycles after done signal
//      before reading hierarchy, ensuring last NB-assign has settled.
//
//  Memory map (word addresses):
//    0x0000 .. 0x0FFF   input   (4096 words)
//    0x1000 .. 0x1008   weights ( 9 words)
//    0x1400 .. 0x2213   output  (3600 words)
//    Total: 8720 words → MEM_WORDS=16384
// ============================================================================
// ============================================================================
//  tb_cnn_accel.v  -  Testbench for CNN_TOP (Single-Port Memory Version)
//
//  Config: 64x64 image, dilation=2
//    OUT_W = 60  →  3600 output pixels
//
//  Changes from multi-port version:
//   1. PEs run sequentially → conv timeout increased to 500000 cycles
//   2. WAIT_MAC state = 10 cycles added between SEQ_PE and WB_REQ
//   3. Per-batch cycles = 8 PEs × ~50 cycles + 10 MAC + 8 WB = ~450
//   4. Total batches = 450, total conv cycles ≈ 450 × 450 = ~200K
// ============================================================================

`timescale 1ns/1ps

module tb_cnn_accel;

// ── Parameters ────────────────────────────────────────────────
parameter DATA_W   = 32;
parameter MEM_AW   = 32;
parameter IMG_W    = 64;
parameter IMG_H    = 64;
parameter DILATION = 2;

localparam OUT_W     = IMG_W - 2*DILATION;   // 60
localparam BUF_DEPTH = IMG_W * IMG_H;        // 4096
localparam OUT_DEPTH = OUT_W * OUT_W;        // 3600
localparam MEM_WORDS = 16384;

localparam [MEM_AW-1:0] INPUT_BASE  = 32'h0000_0000;
localparam [MEM_AW-1:0] WEIGHT_BASE = 32'h0000_4000;
localparam [MEM_AW-1:0] OUTPUT_BASE = 32'h0000_5000;

// ── Clock / reset ─────────────────────────────────────────────
reg clk = 0;
reg rst = 1;
always #5 clk = ~clk;  // 100 MHz

// ── DUT signals ───────────────────────────────────────────────
reg        start      = 0;
reg  [2:0] opcode_top = 0;

reg [MEM_AW-1:0] input_start_addr  = INPUT_BASE;
reg [MEM_AW-1:0] weight_start_addr = WEIGHT_BASE;
reg [MEM_AW-1:0] output_start_addr = OUTPUT_BASE;

reg [7:0] input_channel = 8'd1;
reg [4:0] dilation      = DILATION;

wire             mem_req;
wire             mem_we;
wire [MEM_AW-1:0] mem_addr;
wire [DATA_W-1:0] mem_wdata;
reg  [DATA_W-1:0] mem_rdata_r;
reg               mem_gnt_r;
reg               mem_rvalid_r;

wire load_input_done;
wire load_weight_done;
wire conv_done;
wire write_back_done;
wire cnn_busy;

// ── DUT instantiation ─────────────────────────────────────────
CNN_TOP #(
    .DATA_W(DATA_W), .MEM_AW(MEM_AW), .PE(8),
    .IMG_W(IMG_W),   .IMG_H(IMG_H)
) dut (
    .clk(clk), .rst(rst),
    .start(start), .opcode_top(opcode_top),
    .input_start_addr(input_start_addr),
    .weight_start_addr(weight_start_addr),
    .output_start_addr(output_start_addr),
    .input_channel(input_channel),
    .dilation(dilation),
    .mem_req_o(mem_req),
    .mem_we_o(mem_we),
    .mem_addr_o(mem_addr),
    .mem_wdata_o(mem_wdata),
    .mem_rdata_i(mem_rdata_r),
    .mem_gnt_i(mem_gnt_r),
    .mem_rvalid_i(mem_rvalid_r),
    .load_input_done(load_input_done),
    .load_weight_done(load_weight_done),
    .conv_done(conv_done),
    .write_back_done(write_back_done),
    .cnn_busy(cnn_busy)
);

// ============================================================================
//  Memory Model
//  - Combinational grant
//  - 1-cycle read latency
//  - Writes commit on grant
// ============================================================================
reg [DATA_W-1:0] mem [0:MEM_WORDS-1];

// Grant immediately
always @(*) mem_gnt_r = mem_req;

// 1-cycle read latency
reg          pending_read;
reg [MEM_AW-1:0] pending_addr;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        pending_read  <= 0;
        pending_addr  <= 0;
        mem_rvalid_r  <= 0;
        mem_rdata_r   <= 0;
    end else begin
        // Track read requests
        pending_read <= mem_req & ~mem_we & mem_gnt_r;
        pending_addr <= mem_addr;

        // Return data 1 cycle after grant
        if (pending_read) begin
            mem_rvalid_r <= 1;
            mem_rdata_r  <= mem[pending_addr >> 2];
        end else begin
            mem_rvalid_r <= 0;
            mem_rdata_r  <= 0;
        end

        // Handle writes
        if (mem_req & mem_we & mem_gnt_r)
            mem[mem_addr >> 2] <= mem_wdata;
    end
end

// ============================================================================
//  Helper tasks
// ============================================================================
task send_start;
    input [2:0] op;
    begin
        @(posedge clk); #1;
        opcode_top = op;
        start = 1;
        @(posedge clk); #1;
        start = 0;
    end
endtask

// ============================================================================
//  Golden reference function
//  All-ones kernel: output = sum of 9 dilated window pixels
// ============================================================================
function [31:0] golden;
    input integer row, col, dil;
    integer kr, kc, s;
    begin
        s = 0;
        for (kr = 0; kr < 3; kr = kr+1)
            for (kc = 0; kc < 3; kc = kc+1)
                s = s + ((row + kr*dil)*IMG_W + (col + kc*dil));
        golden = s[31:0];
    end
endfunction

// ============================================================================
//  Verify tasks
// ============================================================================
integer berr;

task verify_input_buffer;
    integer br, bc, bflat, bexp, bact;
    begin
        repeat(2) @(posedge clk);
        berr = 0;
        $display("[TB] Verifying input_buffer[0..%0d]...", BUF_DEPTH-1);
        for (br = 0; br < IMG_H; br = br+1) begin
            for (bc = 0; bc < IMG_W; bc = bc+1) begin
                bflat = br*IMG_W + bc;
                bexp  = bflat;
                bact  = dut.input_buffer[bflat];
                if (bact !== bexp) begin
                    $display("[BUF MISMATCH] [%0d][%0d] exp=%0d got=%0d",
                             br, bc, bexp, bact);
                    berr = berr + 1;
                    if (berr == 10) begin br = IMG_H; bc = IMG_W; end
                end
            end
        end
        if (berr == 0)
            $display("[TB] input_buffer PASS - all %0d pixels correct.", BUF_DEPTH);
        else
            $display("[TB] input_buffer FAIL - %0d mismatches.", berr);
    end
endtask

task verify_weights;
    integer wi, wok;
    begin
        wok = 1;
        $display("[TB] Verifying weight_reg[0..8]...");
        for (wi = 0; wi < 9; wi = wi+1) begin
            if (dut.weight_reg[wi] !== 32'd1) begin
                $display("[W FAIL] weight_reg[%0d] = %0d (expected 1)",
                         wi, dut.weight_reg[wi]);
                wok = 0;
            end
        end
        if (wok) $display("[TB] Weights PASS - all 9 = 1.");
    end
endtask

// ============================================================================
//  Main test sequence
// ============================================================================
integer r, c, widx, timeout;
integer expected, actual, out_widx;
integer errors, checks, pass_cnt;
reg [1:0] phase;

initial begin
    errors   = 0;
    checks   = 0;
    pass_cnt = 0;
    phase    = 0;

    $display("============================================================");
    $display("  CNN_TOP Single-Port Testbench");
    $display("  Image     : %0dx%0d   dilation=%0d", IMG_W, IMG_H, DILATION);
    $display("  Output    : %0dx%0d = %0d pixels", OUT_W, OUT_W, OUT_DEPTH);
    $display("  INPUT_BASE  = 0x%08h", INPUT_BASE);
    $display("  WEIGHT_BASE = 0x%08h", WEIGHT_BASE);
    $display("  OUTPUT_BASE = 0x%08h", OUTPUT_BASE);
    $display("  NOTE: PEs run sequentially - conv takes longer");
    $display("============================================================");

    // ── Init memory ──────────────────────────────────────────────
    for (r = 0; r < IMG_H; r = r+1)
        for (c = 0; c < IMG_W; c = c+1)
            mem[(INPUT_BASE>>2) + r*IMG_W + c] = r*IMG_W + c;

    for (widx = 0; widx < 9; widx = widx+1)
        mem[(WEIGHT_BASE>>2) + widx] = 32'd1;

    for (r = 0; r < MEM_WORDS; r = r+1)
        if (r >= (OUTPUT_BASE>>2))
            mem[r] = 32'hDEAD_BEEF;

    // ── Reset ─────────────────────────────────────────────────────
    rst = 1;
    repeat(5) @(posedge clk); #1;
    rst = 0;
    repeat(2) @(posedge clk);
    $display("[TB] Reset released.");

    // =================================================================
    //  PHASE 1: LOAD INPUT (opcode 001)
    //  4096 pixels × 4 cycles = 16384 min cycles
    //  Timeout: 25000
    // =================================================================
    phase = 1;
    $display("[TB] ── Phase 1: Load Input ──────────────────────────────");
    $display("[TB]    %0d pixels from 0x%08h", BUF_DEPTH, INPUT_BASE);
    send_start(3'b001);

    timeout = 0;
    while (!load_input_done && timeout < 25000) begin
        @(posedge clk);
        timeout = timeout + 1;
    end

    if (!load_input_done) begin
        $display("[FAIL] load_input_done timeout after %0d cycles", timeout);
        $finish;
    end
    $display("[TB]    DONE  t=%0t ns  cycles=%0d", $time, timeout);
    verify_input_buffer;
    repeat(3) @(posedge clk);

    // =================================================================
    //  PHASE 2: LOAD WEIGHTS (opcode 010)
    //  9 weights × 4 cycles = 36 cycles
    //  Timeout: 200
    // =================================================================
    phase = 2;
    $display("[TB] ── Phase 2: Load Weights ────────────────────────────");
    $display("[TB]    9 weights from 0x%08h", WEIGHT_BASE);
    send_start(3'b010);

    timeout = 0;
    while (!load_weight_done && timeout < 200) begin
        @(posedge clk);
        timeout = timeout + 1;
    end

    if (!load_weight_done) begin
        $display("[FAIL] load_weight_done timeout after %0d cycles", timeout);
        $finish;
    end
    $display("[TB]    DONE  t=%0t ns  cycles=%0d", $time, timeout);
    verify_weights;
    repeat(3) @(posedge clk);

    // =================================================================
    //  PHASE 3: CONVOLUTION (opcode 011)
    //  Sequential PEs: 450 batches × 8 PEs × ~50 cycles = ~180K cycles
    //  Plus MAC wait (10) + WB (8) per batch
    //  Timeout: 500000
    // =================================================================
    phase = 3;
    $display("[TB] ── Phase 3: Convolution ─────────────────────────────");
    $display("[TB]    output → 0x%08h  dilation=%0d", OUTPUT_BASE, DILATION);
    $display("[TB]    NOTE: Sequential PEs - approx 200K cycles");
    send_start(3'b011);

    timeout = 0;
    while (!conv_done && timeout < 500000) begin
        @(posedge clk);
        timeout = timeout + 1;
    end

    if (!conv_done) begin
        $display("[FAIL] conv_done timeout after %0d cycles", timeout);
        $finish;
    end
    $display("[TB]    DONE  t=%0t ns  cycles=%0d", $time, timeout);
    repeat(2) @(posedge clk);

    // =================================================================
    //  PHASE 4: Verify output
    // =================================================================
    phase = 0;
    $display("[TB] ── Phase 4: Output Verification ────────────────────");
    $display("[TB]    checking %0d pixels", OUT_DEPTH);
    errors = 0; checks = 0; pass_cnt = 0;

    for (r = 0; r < OUT_W; r = r+1) begin
        for (c = 0; c < OUT_W; c = c+1) begin
            out_widx = (OUTPUT_BASE>>2) + r*OUT_W + c;
            actual   = mem[out_widx];
            expected = golden(r, c, DILATION);
            checks   = checks + 1;
            if (actual === expected) begin
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("[MISMATCH] out[%0d][%0d]  exp=%0d  got=%0d",
                         r, c, expected, actual);
                errors = errors + 1;
                if (errors == 20) begin
                    $display("[TB] Truncated at 20 mismatches.");
                    r = OUT_W; c = OUT_W;
                end
            end
        end
    end

    $display("============================================================");
    $display("  Checked : %0d / %0d pixels", checks, OUT_DEPTH);
    if (errors == 0)
        $display("  RESULT  : PASS - all %0d values correct.", checks);
    else
        $display("  RESULT  : FAIL - %0d mismatches.", errors);
    $display("============================================================");
    $finish;
end

// ============================================================================
//  Watchdog
// ============================================================================
initial begin
    #500_000_000;
    $display("[WATCHDOG] Simulation limit reached. Aborting.");
    $finish;
end

// ============================================================================
//  Transaction monitor (reads suppressed during bulk load)
// ============================================================================
always @(posedge clk) begin
    if (mem_req & ~mem_we & mem_gnt_r & (phase !== 2'd1))
        $display("[MEM RD] t=%0t  addr=0x%08h  data=0x%08h",
                 $time, mem_addr, mem[mem_addr>>2]);
    if (mem_req & mem_we & mem_gnt_r)
        $display("[MEM WR] t=%0t  addr=0x%08h  data=%0d",
                 $time, mem_addr, mem_wdata);
end

// ============================================================================
//  Progress ticker every 50000 cycles during conv
// ============================================================================
integer tick = 0;
always @(posedge clk) begin
    if (phase == 2'd3) begin
        tick = tick + 1;
        if (tick % 50000 == 0)
            $display("[TICK] conv cycle=%0d  busy=%0b  done=%0b  active_pe=%0d",
                     tick, cnn_busy, conv_done, dut.active_pe);
    end else
        tick = 0;
end

endmodule
