// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Execute stage                                              //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Execution stage: Hosts ALU and MAC unit                    //
//                 ALU: computes additions/subtractions/comparisons           //
//                 MULT: computes normal multiplications                      //
//                 APU_DISP: offloads instructions to the shared unit.        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40p_ex_stage
  import cv32e40p_pkg::*;
  import cv32e40p_apu_core_pkg::*;
#(
    parameter FPU              = 0,
    parameter APU_NARGS_CPU    = 3,
    parameter APU_WOP_CPU      = 6,
    parameter APU_NDSFLAGS_CPU = 15,
    parameter APU_NUSFLAGS_CPU = 5
) (
    input logic clk,
    input logic rst_n,

    // ALU signals from ID stage
    input alu_opcode_e        alu_operator_i,
    input logic        [31:0] alu_operand_a_i,
    input logic        [31:0] alu_operand_b_i,
    input logic        [31:0] alu_operand_c_i,
    input logic               alu_en_i,
    input logic        [ 4:0] bmask_a_i,
    input logic        [ 4:0] bmask_b_i,
    input logic        [ 1:0] imm_vec_ext_i,
    input logic        [ 1:0] alu_vec_mode_i,
    input logic               alu_is_clpx_i,
    input logic               alu_is_subrot_i,
    input logic        [ 1:0] alu_clpx_shift_i,

    // Multiplier signals
    input mul_opcode_e        mult_operator_i,
    input logic        [31:0] mult_operand_a_i,
    input logic        [31:0] mult_operand_b_i,
    input logic        [31:0] mult_operand_c_i,
    input logic               mult_en_i,
    input logic               mult_sel_subword_i,
    input logic        [ 1:0] mult_signed_mode_i,
    input logic        [ 4:0] mult_imm_i,

    input logic [31:0] mult_dot_op_a_i,
    input logic [31:0] mult_dot_op_b_i,
    input logic [31:0] mult_dot_op_c_i,
    input logic [ 1:0] mult_dot_signed_i,
    input logic        mult_is_clpx_i,
    input logic [ 1:0] mult_clpx_shift_i,
    input logic        mult_clpx_img_i,

    output logic mult_multicycle_o,

    // FPU signals
    output logic fpu_fflags_we_o,

    //test_tag_mac
    input logic mac_op_en_i,
    input logic [MAC_OP_WIDTH-1:0]mac_operator_i,
    input logic [31:0]	mac_operand_i1,
    input logic [31:0]	mac_operand_i2,
    output logic [31:0]	mac_op_result,
    //test_tag_mac
    //test_tag_con
    input logic [31:0]	con_data_cnt,
    input logic [31:0]	mem_rdata,
    output logic			con_active,
    output logic [1:0]  mac_flag,
    //test_tag_con
    //test_tag_wb
    output logic 		wb23_active,
    output logic		w_wb_active,
    output logic		wb_finish,
    output logic [31:0]	mem_wdata,  
    //test_tag_wb
    //test_tag_mp
    output logic    mp_wb_active_o,
    output logic    mp_ri_active_o,
    //test_tag_mp
    //test_tag_debug
    output logic [67:0] y0,y1,y2,y3,
    output logic [31:0] con_data [15:0],
    //test_tag_debug
    //test_tag_reuse
    output logic con_model,


    // APU signals
    input logic                              apu_en_i,
    input logic [     APU_WOP_CPU-1:0]       apu_op_i,
    input logic [                 1:0]       apu_lat_i,
    input logic [   APU_NARGS_CPU-1:0][31:0] apu_operands_i,
    input logic [                 5:0]       apu_waddr_i,
    input logic [APU_NDSFLAGS_CPU-1:0]       apu_flags_i,

    input  logic [2:0][5:0] apu_read_regs_i,
    input  logic [2:0]      apu_read_regs_valid_i,
    output logic            apu_read_dep_o,
    input  logic [1:0][5:0] apu_write_regs_i,
    input  logic [1:0]      apu_write_regs_valid_i,
    output logic            apu_write_dep_o,

    output logic apu_perf_type_o,
    output logic apu_perf_cont_o,
    output logic apu_perf_wb_o,

    output logic apu_busy_o,
    output logic apu_ready_wb_o,

    // apu-interconnect
    // handshake signals
    output logic                           apu_req_o,
    input  logic                           apu_gnt_i,
    // request channel
    output logic [APU_NARGS_CPU-1:0][31:0] apu_operands_o,
    output logic [  APU_WOP_CPU-1:0]       apu_op_o,
    // response channel
    input  logic                           apu_rvalid_i,
    input  logic [             31:0]       apu_result_i,

    input logic        lsu_en_i,
    input logic [31:0] lsu_rdata_i,

    // input from ID stage
    input logic       branch_in_ex_i,
    input logic [5:0] regfile_alu_waddr_i,
    input logic       regfile_alu_we_i,

    // directly passed through to WB stage, not used in EX
    input logic       regfile_we_i,
    input logic [5:0] regfile_waddr_i,

    // CSR access
    input logic        csr_access_i,
    input logic [31:0] csr_rdata_i,

    // Output of EX stage pipeline
    output logic [ 5:0] regfile_waddr_wb_o,
    output logic        regfile_we_wb_o,
    output logic [31:0] regfile_wdata_wb_o,

    // Forwarding ports : to ID stage
    output logic [ 5:0] regfile_alu_waddr_fw_o,
    output logic        regfile_alu_we_fw_o,
    output logic [31:0] regfile_alu_wdata_fw_o,  // forward to RF and ID/EX pipe, ALU & MUL

    // To IF: Jump and branch target and decision
    output logic [31:0] jump_target_o,
    output logic        branch_decision_o,

    // Stall Control
    input logic         is_decoding_i, // Used to mask data Dependency inside the APU dispatcher in case of an istruction non valid
    input logic lsu_ready_ex_i,  // EX part of LSU is done
    input logic lsu_err_i,

    output logic ex_ready_o,  // EX stage ready for new data
    output logic ex_valid_o,  // EX stage gets new data
    input  logic wb_ready_i,  // WB stage ready for new data
    
    ////////////Custom Signal_CNN_TOP
      input logic       cnn_en_i,         // registered, from ID/EX pipeline  
    // ── CNN Accelerator interface ──────────────────────────────────
input  logic [2:0]  cnn_opcode_i,          // funct3 from decoder
//input  logic [31:0] cnn_in_data_i,         // streaming data (temp, until LSU)
//input  logic [15:0] cnn_input_size_i,      // image width/height
//input  logic [7:0]  cnn_input_channel_i,   // number of channels


// CNN memory bus - connects to arbiter in core
output logic        cnn_mem_req_o,
output logic        cnn_mem_we_o,
output logic [31:0] cnn_mem_addr_o,
output logic [31:0] cnn_mem_wdata_o,
input  logic [31:0] cnn_mem_rdata_i,
input  logic        cnn_mem_gnt_i,
input  logic        cnn_mem_rvalid_i,
input  logic [7:0]  cnn_input_channel_i,   // keep this

output logic cnn_busy_o,
output logic        cnn_write_back_done_o,
output logic        cnn_load_input_done_o,
output logic        cnn_load_weight_done_o,
output logic        cnn_conv_done_o
    
    
    //////////////////////
    
    
    
    
);

  //test_tag_mac
  logic mac_op_ready;
  //test_tag_mac

  logic [31:0] alu_result;
  logic [31:0] mult_result;
  logic        alu_cmp_result;

  logic        regfile_we_lsu;
  logic [ 5:0] regfile_waddr_lsu;

  logic        wb_contention;
  logic        wb_contention_lsu;

  logic        alu_ready;
  logic        mult_ready;

  // APU signals
  logic        apu_valid;
  logic [ 5:0] apu_waddr;
  logic [31:0] apu_result;
  logic        apu_stall;
  logic        apu_active;
  logic        apu_singlecycle;
  logic        apu_multicycle;
  logic        apu_req;
  logic        apu_gnt;
  logic cnn_busy;

  // ALU write port mux
  always_comb begin
    regfile_alu_wdata_fw_o = '0;
    regfile_alu_waddr_fw_o = '0;
    regfile_alu_we_fw_o    = '0;
    wb_contention          = 1'b0;

    // APU single cycle operations, and multicycle operations (>2cycles) are written back on ALU port
    if (apu_valid & (apu_singlecycle | apu_multicycle)) begin
      regfile_alu_we_fw_o    = 1'b1;
      regfile_alu_waddr_fw_o = apu_waddr;
      regfile_alu_wdata_fw_o = apu_result;

      if (regfile_alu_we_i & ~apu_en_i) begin
        wb_contention = 1'b1;
      end
    end else begin
      regfile_alu_we_fw_o    = regfile_alu_we_i & ~apu_en_i;  // private fpu incomplete?
      regfile_alu_waddr_fw_o = regfile_alu_waddr_i;
      if (alu_en_i) regfile_alu_wdata_fw_o = alu_result;
      if (mult_en_i) regfile_alu_wdata_fw_o = mult_result;
      if (csr_access_i) regfile_alu_wdata_fw_o = csr_rdata_i;
    //test_tag_mac
	  if (mac_op_en_i)
		regfile_alu_wdata_fw_o = mac_op_result;
	  //test_tag_mac
    
    end
  end

  // LSU write port mux
  always_comb begin
    regfile_we_wb_o    = 1'b0;
    regfile_waddr_wb_o = regfile_waddr_lsu;
    regfile_wdata_wb_o = lsu_rdata_i;
    wb_contention_lsu  = 1'b0;

    if (regfile_we_lsu) begin
      regfile_we_wb_o = 1'b1;
      if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
        wb_contention_lsu = 1'b1;
      end
      // APU two-cycle operations are written back on LSU port
    end else if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
      regfile_we_wb_o    = 1'b1;
      regfile_waddr_wb_o = apu_waddr;
      regfile_wdata_wb_o = apu_result;
    end
  end

  // branch handling
  assign branch_decision_o = alu_cmp_result;
  assign jump_target_o     = alu_operand_c_i;

  //test_tag_mac
  //test_tag_con 
  
  cv32e40p_mac_ops cv32e40p_mac_ops_i
  (
  .clk			(clk),
  .rst_n		(rst_n),
  .enable_i		(mac_op_en_i),
  .operator_i	(mac_operator_i),
  .operand_i1	(mac_operand_i1),
  .operand_i2	(mac_operand_i2),
  .result_o		(mac_op_result),
  .ready_o		(mac_op_ready),
  .ex_ready_i	(ex_ready_o),

  .con_data_cnt (con_data_cnt),
  .mem_rdata	(mem_rdata),
  .mem_wdata	(mem_wdata),
  .wb23_active	(wb23_active),
  .w_wb_active	(w_wb_active),
  .wb_finish	(wb_finish),
  .con_active_o	(con_active),
  .mp_wb_active_o (mp_wb_active_o),
  .mp_ri_active_o (mp_ri_active_o),
  .mac_flag (mac_flag),
  .con_model(con_model),
  .y0_o         (y0),
  .y1_o         (y1),
  .y2_o         (y2),
  .y3_o         (y3),
  .con_data     (con_data)
  );

  //test_tag_con
  //test_tag_mac

  ////////////////////////////
  //     _    _    _   _    //
  //    / \  | |  | | | |   //
  //   / _ \ | |  | | | |   //
  //  / ___ \| |__| |_| |   //
  // /_/   \_\_____\___/    //
  //                        //
  ////////////////////////////

  cv32e40p_alu alu_i (
      .clk        (clk),
      .rst_n      (rst_n),
      .enable_i   (alu_en_i),
      .operator_i (alu_operator_i),
      .operand_a_i(alu_operand_a_i),
      .operand_b_i(alu_operand_b_i),
      .operand_c_i(alu_operand_c_i),

      .vector_mode_i(alu_vec_mode_i),
      .bmask_a_i    (bmask_a_i),
      .bmask_b_i    (bmask_b_i),
      .imm_vec_ext_i(imm_vec_ext_i),

      .is_clpx_i   (alu_is_clpx_i),
      .clpx_shift_i(alu_clpx_shift_i),
      .is_subrot_i (alu_is_subrot_i),

      .result_o           (alu_result),
      .comparison_result_o(alu_cmp_result),

      .ready_o   (alu_ready),
      .ex_ready_i(ex_ready_o)
  );


  ////////////////////////////////////////////////////////////////
  //  __  __ _   _ _   _____ ___ ____  _     ___ _____ ____     //
  // |  \/  | | | | | |_   _|_ _|  _ \| |   |_ _| ____|  _ \    //
  // | |\/| | | | | |   | |  | || |_) | |    | ||  _| | |_) |   //
  // | |  | | |_| | |___| |  | ||  __/| |___ | || |___|  _ <    //
  // |_|  |_|\___/|_____|_| |___|_|   |_____|___|_____|_| \_\   //
  //                                                            //
  ////////////////////////////////////////////////////////////////

  cv32e40p_mult mult_i (
      .clk  (clk),
      .rst_n(rst_n),

      .enable_i  (mult_en_i),
      .operator_i(mult_operator_i),

      .short_subword_i(mult_sel_subword_i),
      .short_signed_i (mult_signed_mode_i),

      .op_a_i(mult_operand_a_i),
      .op_b_i(mult_operand_b_i),
      .op_c_i(mult_operand_c_i),
      .imm_i (mult_imm_i),

      .dot_op_a_i  (mult_dot_op_a_i),
      .dot_op_b_i  (mult_dot_op_b_i),
      .dot_op_c_i  (mult_dot_op_c_i),
      .dot_signed_i(mult_dot_signed_i),
      .is_clpx_i   (mult_is_clpx_i),
      .clpx_shift_i(mult_clpx_shift_i),
      .clpx_img_i  (mult_clpx_img_i),

      .result_o(mult_result),

      .multicycle_o(mult_multicycle_o),
      .ready_o     (mult_ready),
      .ex_ready_i  (ex_ready_o)
  );

  generate
    if (FPU == 1) begin : gen_apu
      ////////////////////////////////////////////////////
      //     _    ____  _   _   ____ ___ ____  ____     //
      //    / \  |  _ \| | | | |  _ \_ _/ ___||  _ \    //
      //   / _ \ | |_) | | | | | | | | |\___ \| |_) |   //
      //  / ___ \|  __/| |_| | | |_| | | ___) |  __/    //
      // /_/   \_\_|    \___/  |____/___|____/|_|       //
      //                                                //
      ////////////////////////////////////////////////////

      cv32e40p_apu_disp apu_disp_i (
          .clk_i (clk),
          .rst_ni(rst_n),

          .enable_i   (apu_en_i),
          .apu_lat_i  (apu_lat_i),
          .apu_waddr_i(apu_waddr_i),

          .apu_waddr_o      (apu_waddr),
          .apu_multicycle_o (apu_multicycle),
          .apu_singlecycle_o(apu_singlecycle),

          .active_o(apu_active),
          .stall_o (apu_stall),

          .is_decoding_i     (is_decoding_i),
          .read_regs_i       (apu_read_regs_i),
          .read_regs_valid_i (apu_read_regs_valid_i),
          .read_dep_o        (apu_read_dep_o),
          .write_regs_i      (apu_write_regs_i),
          .write_regs_valid_i(apu_write_regs_valid_i),
          .write_dep_o       (apu_write_dep_o),

          .perf_type_o(apu_perf_type_o),
          .perf_cont_o(apu_perf_cont_o),

          // apu-interconnect
          // handshake signals
          .apu_req_o   (apu_req),
          .apu_gnt_i   (apu_gnt),
          // response channel
          .apu_rvalid_i(apu_valid)
      );

      assign apu_perf_wb_o   = wb_contention | wb_contention_lsu;
      assign apu_ready_wb_o  = ~(apu_active | apu_en_i | apu_stall) | apu_valid;

      assign apu_req_o       = apu_req;
      assign apu_gnt         = apu_gnt_i;
      assign apu_valid       = apu_rvalid_i;
      assign apu_operands_o  = apu_operands_i;
      assign apu_op_o        = apu_op_i;
      assign apu_result      = apu_result_i;
      assign fpu_fflags_we_o = apu_valid;
    end else begin : gen_no_apu
      // default assignements for the case when no FPU/APU is attached.
      assign apu_req_o         = '0;
      assign apu_operands_o[0] = '0;
      assign apu_operands_o[1] = '0;
      assign apu_operands_o[2] = '0;
      assign apu_op_o          = '0;
      assign apu_req           = 1'b0;
      assign apu_gnt           = 1'b0;
      assign apu_result        = 32'b0;
      assign apu_valid         = 1'b0;
      assign apu_waddr         = 6'b0;
      assign apu_stall         = 1'b0;
      assign apu_active        = 1'b0;
      assign apu_ready_wb_o    = 1'b1;
      assign apu_perf_wb_o     = 1'b0;
      assign apu_perf_cont_o   = 1'b0;
      assign apu_perf_type_o   = 1'b0;
      assign apu_singlecycle   = 1'b0;
      assign apu_multicycle    = 1'b0;
      assign apu_read_dep_o    = 1'b0;
      assign apu_write_dep_o   = 1'b0;
      assign fpu_fflags_we_o   = 1'b0;

    end
  endgenerate

  assign apu_busy_o = apu_active;

  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n) begin : EX_WB_Pipeline_Register
    if (~rst_n) begin
      regfile_waddr_lsu <= '0;
      regfile_we_lsu    <= 1'b0;
    end else begin
      if (ex_valid_o) // wb_ready_i is implied
      begin
        regfile_we_lsu <= regfile_we_i & ~lsu_err_i;
        if (regfile_we_i & ~lsu_err_i) begin
          regfile_waddr_lsu <= regfile_waddr_i;
        end
      end else if (wb_ready_i) begin
        // we are ready for a new instruction, but there is none available,
        // so we just flush the current one out of the pipe
        regfile_we_lsu <= 1'b0;
      end
    end
  end

  // As valid always goes to the right and ready to the left, and we are able
  // to finish branches without going to the WB stage, ex_valid does not
  // depend on ex_ready.
  //ori
  //assign ex_ready_o = (~apu_stall & alu_ready & mult_ready & lsu_ready_ex_i
  //                     & wb_ready_i & ~wb_contention) | (branch_in_ex_i);
  //ori
  assign ex_valid_o = (apu_valid | alu_en_i | mult_en_i | csr_access_i | lsu_en_i)
                       & (alu_ready & mult_ready & lsu_ready_ex_i & wb_ready_i);
  //test_tag_mac
  assign ex_ready_o = (~apu_stall & alu_ready & mult_ready & lsu_ready_ex_i
                    & wb_ready_i & ~wb_contention & mac_op_ready & ~cnn_busy) | (branch_in_ex_i);
  //test_tag_mac
// ── CNN_TOP internal wires ─────────────────────────────────────────
//logic [11:0] cnn_out_addr;
//logic [31:0] cnn_out_val;
// ── CNN memory bus wires ───────────────────────────────────────────
logic        cnn_mem_req;
logic        cnn_mem_we;
logic [31:0] cnn_mem_addr;
logic [31:0] cnn_mem_wdata;

logic        cnn_write_back_done;
logic        cnn_load_input_done;
logic        cnn_load_weight_done;
logic        cnn_conv_done;

assign cnn_mem_req_o   = cnn_mem_req;
assign cnn_mem_we_o    = cnn_mem_we;
assign cnn_mem_addr_o  = cnn_mem_addr;
assign cnn_mem_wdata_o = cnn_mem_wdata;



// Tie outputs to ports

assign cnn_write_back_done_o  = cnn_write_back_done;
assign cnn_load_input_done_o  = cnn_load_input_done;
assign cnn_load_weight_done_o = cnn_load_weight_done;
assign cnn_conv_done_o        = cnn_conv_done;

assign cnn_busy_o = cnn_busy;

CNN_TOP #(
    .DATA_W (32),
    .MEM_AW (32),
    .PE     (8),
    .IMG_W  (64),
    .IMG_H  (64)
) cnn_accel_i (
    .clk               (clk),
    .rst               (~rst_n),
    .start             (cnn_en_i),
    .opcode_top        (cnn_opcode_i),

    // rs1 → input base addr (for load_input)
    // rs1 → weight base addr (for load_weight)
    // rs1 → input base addr (for conv)
    .input_start_addr  (alu_operand_a_i),
    .weight_start_addr (alu_operand_a_i),   // same register for load_weight
    .output_start_addr (alu_operand_b_i),   // rs2

    .input_channel     (cnn_input_channel_i),
    .dilation          (alu_operand_c_i[4:0]),  // rs3

    // Memory bus → goes to arbiter in core
    .mem_req_o         (cnn_mem_req),
    .mem_we_o          (cnn_mem_we),
    .mem_addr_o        (cnn_mem_addr),
    .mem_wdata_o       (cnn_mem_wdata),
    .mem_rdata_i       (cnn_mem_rdata_i),
    .mem_gnt_i         (cnn_mem_gnt_i),
    .mem_rvalid_i      (cnn_mem_rvalid_i),

    // Status
    .load_input_done   (cnn_load_input_done),
.load_weight_done  (cnn_load_weight_done),
.conv_done         (cnn_conv_done),
.write_back_done   (cnn_write_back_done),
    .cnn_busy          (cnn_busy)
);

endmodule
module CNN_TOP
#(
    parameter DATA_W  = 32,
    parameter MEM_AW  = 32,
    parameter PE      = 8,
    parameter IMG_W   = 64,
    parameter IMG_H   = 64
)
(
    input  wire clk, rst,
    input  wire        start,
    input  wire [2:0]  opcode_top,
    input  wire [MEM_AW-1:0] input_start_addr,
    input  wire [MEM_AW-1:0] weight_start_addr,
    input  wire [MEM_AW-1:0] output_start_addr,
    input  wire [7:0]  input_channel,
    input  wire [4:0]  dilation,
    output reg             mem_req_o,
    output reg             mem_we_o,
    output reg [MEM_AW-1:0] mem_addr_o,
    output reg [DATA_W-1:0] mem_wdata_o,
    input  wire [DATA_W-1:0] mem_rdata_i,
    input  wire              mem_gnt_i,
    input  wire              mem_rvalid_i,
    output reg  load_input_done,
    output reg  load_weight_done,
    output reg  conv_done,
    output reg  write_back_done,
    output reg  cnn_busy
);

localparam BUF_DEPTH = IMG_W * IMG_H;
localparam BUF_AW    = $clog2(BUF_DEPTH);

// ── Single port memory ────────────────────────────────────────
reg [DATA_W-1:0] input_buffer [0:BUF_DEPTH-1];
reg [DATA_W-1:0] weight_reg   [0:8];

// ── FSM states ────────────────────────────────────────────────
localparam [3:0]
    IDLE      = 4'd0,
    LI_REQ    = 4'd1,
    LI_DATA   = 4'd2,
    LW_REQ    = 4'd3,
    LW_DATA   = 4'd4,
    START_CONV= 4'd5,
    SEQ_PE    = 4'd6,   // sequential PE execution
    WAIT_MAC  = 4'd7,   // wait for MAC pipeline to flush
    WB_REQ    = 4'd8,
    NEXT_REQ  = 4'd9,
    DONE      = 4'd10;

reg [3:0] state, next_state;

reg [BUF_AW:0] load_cnt;
reg [3:0]      w_idx;
reg [3:0]      mac_cnt;    // counts MAC pipeline flush cycles
reg [2:0]      wb_cnt;
reg [2:0]      active_pe;  // which PE runs now (0..7)
reg            pe_launched; // has current active_pe been started?

reg  req;
reg  pe_start_r;           // 1-cycle pulse for active PE only
wire base_done;
reg  base_done_latched;

// ── PE wires ──────────────────────────────────────────────────
wire [BUF_AW-1:0] base_addr0,base_addr1,base_addr2,base_addr3;
wire [BUF_AW-1:0] base_addr4,base_addr5,base_addr6,base_addr7;

wire [67:0]       conv_out0,conv_out1,conv_out2,conv_out3;
wire [67:0]       conv_out4,conv_out5,conv_out6,conv_out7;
wire [BUF_AW-1:0] out_addr0,out_addr1,out_addr2,out_addr3;
wire [BUF_AW-1:0] out_addr4,out_addr5,out_addr6,out_addr7;
wire              done0,done1,done2,done3,done4,done5,done6,done7;
wire [BUF_AW-1:0] maddr0,maddr1,maddr2,maddr3;
wire [BUF_AW-1:0] maddr4,maddr5,maddr6,maddr7;

// ── Single port read mux ──────────────────────────────────────
reg  [BUF_AW-1:0] rd_addr;
reg  [DATA_W-1:0] rd_data;

always @(*) begin
    case (active_pe)
        3'd0: rd_addr = maddr0;
        3'd1: rd_addr = maddr1;
        3'd2: rd_addr = maddr2;
        3'd3: rd_addr = maddr3;
        3'd4: rd_addr = maddr4;
        3'd5: rd_addr = maddr5;
        3'd6: rd_addr = maddr6;
        3'd7: rd_addr = maddr7;
        default: rd_addr = maddr0;
    endcase
end

// Registered single-port read (1 cycle latency - matches Fetch WAIT_1)
always @(posedge clk)
    rd_data <= input_buffer[rd_addr];

// Route rd_data to all PEs - only active PE Fetch is running
wire [DATA_W-1:0] mrdata0 = rd_data;
wire [DATA_W-1:0] mrdata1 = rd_data;
wire [DATA_W-1:0] mrdata2 = rd_data;
wire [DATA_W-1:0] mrdata3 = rd_data;
wire [DATA_W-1:0] mrdata4 = rd_data;
wire [DATA_W-1:0] mrdata5 = rd_data;
wire [DATA_W-1:0] mrdata6 = rd_data;
wire [DATA_W-1:0] mrdata7 = rd_data;

// One-hot pe_start - only active PE gets the pulse
wire [7:0] pe_start_vec;
assign pe_start_vec[0] = pe_start_r & (active_pe==3'd0);
assign pe_start_vec[1] = pe_start_r & (active_pe==3'd1);
assign pe_start_vec[2] = pe_start_r & (active_pe==3'd2);
assign pe_start_vec[3] = pe_start_r & (active_pe==3'd3);
assign pe_start_vec[4] = pe_start_r & (active_pe==3'd4);
assign pe_start_vec[5] = pe_start_r & (active_pe==3'd5);
assign pe_start_vec[6] = pe_start_r & (active_pe==3'd6);
assign pe_start_vec[7] = pe_start_r & (active_pe==3'd7);

// Done signal for active PE
wire [7:0] done_vec = {done7,done6,done5,done4,done3,done2,done1,done0};
wire       active_done = done_vec[active_pe];

// ── Writeback addresses ────────────────────────────────────────
wire [MEM_AW-1:0] wb_byte_addr [0:7];
wire [DATA_W-1:0] wb_data      [0:7];

assign wb_byte_addr[0]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr0}<<2);
assign wb_byte_addr[1]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr1}<<2);
assign wb_byte_addr[2]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr2}<<2);
assign wb_byte_addr[3]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr3}<<2);
assign wb_byte_addr[4]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr4}<<2);
assign wb_byte_addr[5]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr5}<<2);
assign wb_byte_addr[6]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr6}<<2);
assign wb_byte_addr[7]=output_start_addr+({{(MEM_AW-BUF_AW){1'b0}},out_addr7}<<2);

assign wb_data[0]=conv_out0[DATA_W-1:0];
assign wb_data[1]=conv_out1[DATA_W-1:0];
assign wb_data[2]=conv_out2[DATA_W-1:0];
assign wb_data[3]=conv_out3[DATA_W-1:0];
assign wb_data[4]=conv_out4[DATA_W-1:0];
assign wb_data[5]=conv_out5[DATA_W-1:0];
assign wb_data[6]=conv_out6[DATA_W-1:0];
assign wb_data[7]=conv_out7[DATA_W-1:0];

// ── FSM state register ────────────────────────────────────────
always @(posedge clk or posedge rst)
    if (rst) state <= IDLE;
    else     state <= next_state;

// ── FSM next-state logic ──────────────────────────────────────
always @(*) begin
    next_state = state;
    case (state)
        IDLE:       if (start)
                        case (opcode_top)
                            3'b001: next_state = LI_REQ;
                            3'b010: next_state = LW_REQ;
                            3'b011: next_state = START_CONV;
                            default:next_state = IDLE;
                        endcase
        LI_REQ:     if (mem_gnt_i)    next_state = LI_DATA;
        LI_DATA:    if (mem_rvalid_i) next_state = (load_cnt==BUF_DEPTH-1) ? DONE : LI_REQ;
        LW_REQ:     if (mem_gnt_i)    next_state = LW_DATA;
        LW_DATA:    if (mem_rvalid_i) next_state = (w_idx==4'd8) ? DONE : LW_REQ;
        START_CONV: next_state = SEQ_PE;

        // Run PE0..PE7 one at a time
        SEQ_PE:     if (active_done && active_pe==3'd7)
                        next_state = WAIT_MAC;

        // Wait for MAC pipeline (5 pipeline stages)
        WAIT_MAC:   if (mac_cnt==4'd10) next_state = WB_REQ;

        WB_REQ:     if (mem_gnt_i && wb_cnt==3'd7) next_state = NEXT_REQ;
        NEXT_REQ:   next_state = base_done_latched ? DONE : START_CONV;
        DONE:       next_state = IDLE;
    endcase
end

// ── FSM datapath ──────────────────────────────────────────────
always @(posedge clk or posedge rst) begin
    if (rst) begin
        mem_req_o       <= 1'b0;
        mem_we_o        <= 1'b0;
        mem_addr_o      <= {MEM_AW{1'b0}};
        mem_wdata_o     <= {DATA_W{1'b0}};
        req             <= 1'b0;
        pe_start_r      <= 1'b0;
        mac_cnt         <= 4'd0;
        wb_cnt          <= 3'd0;
        load_cnt        <= {(BUF_AW+1){1'b0}};
        w_idx           <= 4'd0;
        active_pe       <= 3'd0;
        pe_launched     <= 1'b0;
        weight_reg[0]   <= {DATA_W{1'b0}};
        weight_reg[1]   <= {DATA_W{1'b0}};
        weight_reg[2]   <= {DATA_W{1'b0}};
        weight_reg[3]   <= {DATA_W{1'b0}};
        weight_reg[4]   <= {DATA_W{1'b0}};
        weight_reg[5]   <= {DATA_W{1'b0}};
        weight_reg[6]   <= {DATA_W{1'b0}};
        weight_reg[7]   <= {DATA_W{1'b0}};
        weight_reg[8]   <= {DATA_W{1'b0}};
        base_done_latched  <= 1'b0;
        load_input_done    <= 1'b0;
        load_weight_done   <= 1'b0;
        conv_done          <= 1'b0;
        write_back_done    <= 1'b0;
        cnn_busy           <= 1'b0;
    end else begin
        mem_req_o<=0; mem_we_o<=0;
        req<=0; pe_start_r<=0;

        if (base_done)     base_done_latched <= 1'b1;
        if (state==IDLE)   base_done_latched <= 1'b0;

        case (state)
        IDLE: begin
            load_cnt<=0; w_idx<=0;
            mac_cnt<=0; wb_cnt<=0;
            active_pe<=0; pe_launched<=0;
            write_back_done<=0; conv_done<=0;
            cnn_busy<=start;
        end

        LI_REQ: begin
            cnn_busy<=1; mem_req_o<=1; mem_we_o<=0;
            mem_addr_o <= input_start_addr +
                ({{(MEM_AW-BUF_AW-1){1'b0}},load_cnt[BUF_AW-1:0]}<<2);
        end

        LI_DATA: begin
            if (mem_rvalid_i) begin
                input_buffer[load_cnt] <= mem_rdata_i;
                if (load_cnt==BUF_DEPTH-1) begin
                    load_input_done<=1; load_cnt<=0;
                end else
                    load_cnt<=load_cnt+1;
            end
        end

        LW_REQ: begin
            cnn_busy<=1; mem_req_o<=1; mem_we_o<=0;
            mem_addr_o <= weight_start_addr + ({28'd0,w_idx}<<2);
        end

        LW_DATA: begin
            if (mem_rvalid_i) begin
                weight_reg[w_idx] <= mem_rdata_i;
                if (w_idx==4'd8) begin
                    load_weight_done<=1; w_idx<=0;
                end else
                    w_idx<=w_idx+1;
            end
        end

        START_CONV: begin
            cnn_busy<=1; req<=1;
            active_pe<=0; pe_launched<=0;
            mac_cnt<=0; wb_cnt<=0;
        end

        SEQ_PE: begin
            cnn_busy<=1;
            // Step 1: Launch active PE (1-cycle pulse)
            if (!pe_launched) begin
                pe_start_r  <= 1'b1;
                pe_launched <= 1'b1;
            end
            // Step 2: When active PE finishes, move to next
            if (active_done && pe_launched) begin
                if (active_pe < 3'd7) begin
                    active_pe   <= active_pe + 1;
                    pe_launched <= 1'b0;  // reset for next PE
                end
                // if active_pe==7 → FSM moves to WAIT_MAC
            end
        end

        WAIT_MAC: begin
            cnn_busy<=1;
            mac_cnt <= mac_cnt + 1;
        end

        WB_REQ: begin
            cnn_busy<=1; mem_req_o<=1; mem_we_o<=1;
            mem_addr_o  <= wb_byte_addr[wb_cnt];
            mem_wdata_o <= wb_data[wb_cnt];
            if (mem_gnt_i) wb_cnt <= wb_cnt+1;
        end

        NEXT_REQ: begin
            cnn_busy<=1; write_back_done<=1;
        end

        DONE: begin
            cnn_busy<=0; write_back_done<=1;
            if (opcode_top==3'b011) conv_done<=1;
        end
        endcase
    end
end

// ── Sub-modules ───────────────────────────────────────────────
BaseAddrGen #(.IMG_W(IMG_W),.IMG_H(IMG_H),.ADDR_W(BUF_AW),.PE(PE)) addr_gen (
    .clk(clk),.rst(rst),.req(req),.dilation(dilation),.done(base_done),
    .base_addr0(base_addr0),.base_addr1(base_addr1),
    .base_addr2(base_addr2),.base_addr3(base_addr3),
    .base_addr4(base_addr4),.base_addr5(base_addr5),
    .base_addr6(base_addr6),.base_addr7(base_addr7));

PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe0(.clk(clk),.rst(rst),.start(pe_start_vec[0]),.dilation(dilation),.base_addr(base_addr0),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done0),.conv_out(conv_out0),.mem_addr(maddr0),.mem_rdata(mrdata0),.out_addr(out_addr0));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe1(.clk(clk),.rst(rst),.start(pe_start_vec[1]),.dilation(dilation),.base_addr(base_addr1),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done1),.conv_out(conv_out1),.mem_addr(maddr1),.mem_rdata(mrdata1),.out_addr(out_addr1));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe2(.clk(clk),.rst(rst),.start(pe_start_vec[2]),.dilation(dilation),.base_addr(base_addr2),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done2),.conv_out(conv_out2),.mem_addr(maddr2),.mem_rdata(mrdata2),.out_addr(out_addr2));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe3(.clk(clk),.rst(rst),.start(pe_start_vec[3]),.dilation(dilation),.base_addr(base_addr3),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done3),.conv_out(conv_out3),.mem_addr(maddr3),.mem_rdata(mrdata3),.out_addr(out_addr3));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe4(.clk(clk),.rst(rst),.start(pe_start_vec[4]),.dilation(dilation),.base_addr(base_addr4),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done4),.conv_out(conv_out4),.mem_addr(maddr4),.mem_rdata(mrdata4),.out_addr(out_addr4));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe5(.clk(clk),.rst(rst),.start(pe_start_vec[5]),.dilation(dilation),.base_addr(base_addr5),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done5),.conv_out(conv_out5),.mem_addr(maddr5),.mem_rdata(mrdata5),.out_addr(out_addr5));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe6(.clk(clk),.rst(rst),.start(pe_start_vec[6]),.dilation(dilation),.base_addr(base_addr6),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done6),.conv_out(conv_out6),.mem_addr(maddr6),.mem_rdata(mrdata6),.out_addr(out_addr6));
PE #(.WIDTH(IMG_W),.DATA_W(DATA_W),.ADDR_W(BUF_AW)) pe7(.clk(clk),.rst(rst),.start(pe_start_vec[7]),.dilation(dilation),.base_addr(base_addr7),.w0(weight_reg[0]),.w1(weight_reg[1]),.w2(weight_reg[2]),.w3(weight_reg[3]),.w4(weight_reg[4]),.w5(weight_reg[5]),.w6(weight_reg[6]),.w7(weight_reg[7]),.w8(weight_reg[8]),.done(done7),.conv_out(conv_out7),.mem_addr(maddr7),.mem_rdata(mrdata7),.out_addr(out_addr7));

endmodule

//=============================================================================
// BaseAddrGen
// FIX 2: DONE_S → DONE_S (stays stuck) so done=1 is held indefinitely
//=============================================================================
module BaseAddrGen
#(
    parameter IMG_W  = 64,
    parameter IMG_H  = 64,
    parameter ADDR_W = 12,
    parameter PE     = 8
)
(
    input  wire clk, rst, req,
    input  wire [4:0] dilation,
    output reg done,
    output reg [ADDR_W-1:0] base_addr0,base_addr1,base_addr2,base_addr3,
    output reg [ADDR_W-1:0] base_addr4,base_addr5,base_addr6,base_addr7
);

localparam COL_MSK = IMG_W - 1;
localparam [1:0] IDLE_S=2'd0, RUN_S=2'd1, DONE_S=2'd2;

reg [1:0] state, next_state;
reg [15:0] row, col;

wire [15:0]    dil2      = {11'b0,dilation} << 1;
wire [15:0]    OUT_W     = IMG_W - dil2;
wire [ADDR_W-1:0] last_valid = (IMG_H-1-dil2[4:0])*IMG_W + (IMG_W-1-dil2[5:0]);
wire [ADDR_W-1:0] max_col_v  = IMG_W - dil2[5:0] - 1;

reg [ADDR_W-1:0] n0,n1,n2,n3,n4,n5,n6,n7;

always @(posedge clk or posedge rst)
    if (rst) state <= IDLE_S;
    else     state <= next_state;

always @(*) begin
    next_state = state;
    case (state)
        IDLE_S: if (req)  next_state = RUN_S;
        RUN_S:  if (done) next_state = DONE_S;
        DONE_S:           next_state = DONE_S;  // FIX 2: stay stuck
    endcase
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        row<=0; col<=0; done<=0;
        base_addr0<=0; base_addr1<=0; base_addr2<=0; base_addr3<=0;
        base_addr4<=0; base_addr5<=0; base_addr6<=0; base_addr7<=0;
    end else begin
        case (state)
        IDLE_S: begin row<=0; col<=0; done<=0; end
        RUN_S: begin
            if (req && !done) begin
                n0=row*IMG_W+col; n1=n0+1; n2=n0+2; n3=n0+3;
                n4=n0+4;          n5=n0+5; n6=n0+6; n7=n0+7;

                if(n0<=last_valid&&(n0&COL_MSK)<=max_col_v) base_addr0<=n0[ADDR_W-1:0];
                if(n1<=last_valid&&(n1&COL_MSK)<=max_col_v) base_addr1<=n1[ADDR_W-1:0];
                if(n2<=last_valid&&(n2&COL_MSK)<=max_col_v) base_addr2<=n2[ADDR_W-1:0];
                if(n3<=last_valid&&(n3&COL_MSK)<=max_col_v) base_addr3<=n3[ADDR_W-1:0];
                if(n4<=last_valid&&(n4&COL_MSK)<=max_col_v) base_addr4<=n4[ADDR_W-1:0];
                if(n5<=last_valid&&(n5&COL_MSK)<=max_col_v) base_addr5<=n5[ADDR_W-1:0];
                if(n6<=last_valid&&(n6&COL_MSK)<=max_col_v) base_addr6<=n6[ADDR_W-1:0];
                if(n7<=last_valid&&(n7&COL_MSK)<=max_col_v) base_addr7<=n7[ADDR_W-1:0];

                col<=col+PE;
                if(col+PE>=OUT_W) begin col<=0; row<=row+1; end

                if(n0>last_valid&&n1>last_valid&&n2>last_valid&&n3>last_valid&&
                   n4>last_valid&&n5>last_valid&&n6>last_valid&&n7>last_valid)
                    done<=1;
            end
        end
        DONE_S: done<=1;
        endcase
    end
end
endmodule


//=============================================================================
// PE
//=============================================================================
module PE
#(
    parameter WIDTH  = 64,
    parameter DATA_W = 32,
    parameter ADDR_W = 12
)
(
    input  wire clk, rst, start,
    input  wire [4:0]        dilation,
    input  wire [ADDR_W-1:0] base_addr,
    input  wire [DATA_W-1:0] w0,w1,w2,w3,w4,w5,w6,w7,w8,
    output wire              done,
    output wire [67:0]       conv_out,
    output wire [ADDR_W-1:0] mem_addr,
    input  wire [DATA_W-1:0] mem_rdata,
    output wire [ADDR_W-1:0] out_addr
);

localparam LOGW = $clog2(WIDTH);
localparam CMSK = WIDTH - 1;

wire [DATA_W-1:0] x0,x1,x2,x3,x4,x5,x6,x7,x8;

Fetch #(.WIDTH(WIDTH),.DATA_W(DATA_W),.ADDR_W(ADDR_W)) fetch_inst(
    .clk(clk),.rst(rst),.start(start),.dilation(dilation),.base_addr(base_addr),
    .mem_addr(mem_addr),.mem_rdata(mem_rdata),
    .window0(x0),.window1(x1),.window2(x2),.window3(x3),.window4(x4),
    .window5(x5),.window6(x6),.window7(x7),.window8(x8),.done(done));

MAC9_PIPE mac_inst(
    .clk(clk),.rst(rst),.en(done),
    .x0(x0),.x1(x1),.x2(x2),.x3(x3),.x4(x4),
    .x5(x5),.x6(x6),.x7(x7),.x8(x8),
    .w0(w0),.w1(w1),.w2(w2),.w3(w3),.w4(w4),
    .w5(w5),.w6(w6),.w7(w7),.w8(w8),.y(conv_out));

reg [ADDR_W-1:0] base_addr_reg;
always @(posedge clk) if(start) base_addr_reg<=base_addr;

wire [ADDR_W-1:0] row_pe    = base_addr_reg >> LOGW;
wire [ADDR_W-1:0] col_pe    = base_addr_reg & CMSK;
wire [ADDR_W-1:0] out_width = WIDTH - ({1'b0,dilation}<<1);
assign out_addr = row_pe * out_width + col_pe;

endmodule


//=============================================================================
// Fetch
//=============================================================================
module Fetch
#(
    parameter WIDTH  = 64,
    parameter DATA_W = 32,
    parameter ADDR_W = 12
)
(
    input  wire clk, rst, start,
    input  wire [4:0]        dilation,
    input  wire [ADDR_W-1:0] base_addr,
    output wire [ADDR_W-1:0] mem_addr,
    input  wire [DATA_W-1:0] mem_rdata,
    output wire [DATA_W-1:0] window0,window1,window2,
    output wire [DATA_W-1:0] window3,window4,window5,
    output wire [DATA_W-1:0] window6,window7,window8,
    output reg  done
);

localparam [2:0] IDLE=0,ISSUE=1,WAIT_1=2,CAPTURE=3,UPDATE=4,DONE_S=5;
reg [2:0] state, next_state;
reg [3:0]        issue_idx, cap_idx;
reg [ADDR_W-1:0] addr, row_base, row_step;
reg [DATA_W-1:0] window [0:8];

assign mem_addr=addr;
assign window0=window[0]; assign window1=window[1]; assign window2=window[2];
assign window3=window[3]; assign window4=window[4]; assign window5=window[5];
assign window6=window[6]; assign window7=window[7]; assign window8=window[8];

always @(posedge clk or posedge rst)
    if(rst) state<=IDLE; else state<=next_state;

always @(*) begin
    next_state=state;
    case(state)
        IDLE:    if(start)         next_state=ISSUE;
        ISSUE:                     next_state=WAIT_1;
        WAIT_1:                    next_state=CAPTURE;
        CAPTURE:                   next_state=UPDATE;
        UPDATE:  if(cap_idx==4'd8) next_state=DONE_S;
                 else              next_state=ISSUE;
        DONE_S:                    next_state=IDLE;
        default:                   next_state=IDLE;
    endcase
end

integer fi;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        done<=0; issue_idx<=0; cap_idx<=0;
        addr<={ADDR_W{1'b0}}; row_base<={ADDR_W{1'b0}}; row_step<={ADDR_W{1'b0}};
        for(fi=0;fi<9;fi=fi+1) window[fi]<={DATA_W{1'b0}};
    end else begin
        done<=0;
        case(state)
        IDLE: if(start) begin
            issue_idx<=0; cap_idx<=0;
            row_step<=dilation*WIDTH; row_base<=base_addr; addr<=base_addr;
        end
        ISSUE:;
        WAIT_1:;
        CAPTURE: window[cap_idx]<=mem_rdata;
        UPDATE: begin
            cap_idx<=cap_idx+4'd1; issue_idx<=issue_idx+4'd1;
            if(issue_idx==4'd2||issue_idx==4'd5) begin
                row_base<=row_base+row_step; addr<=row_base+row_step;
            end else
                addr<=addr+dilation;
        end
        DONE_S: done<=1;
        endcase
    end
end
endmodule


//=============================================================================
// MAC9_PIPE
//=============================================================================
module MAC9_PIPE(
    input  wire       clk,rst,en,
    input  wire [31:0] x0,x1,x2,x3,x4,x5,x6,x7,x8,
    input  wire [31:0] w0,w1,w2,w3,w4,w5,w6,w7,w8,
    output reg  [67:0] y
);
reg [63:0] m0,m1,m2,m3,m4,m5,m6,m7,m8;
always @(posedge clk)
    if(en) begin
        m0<=x0*w0; m1<=x1*w1; m2<=x2*w2; m3<=x3*w3; m4<=x4*w4;
        m5<=x5*w5; m6<=x6*w6; m7<=x7*w7; m8<=x8*w8;
    end
reg [64:0] s0,s1,s2,s3;
always @(posedge clk) begin s0<=m0+m1; s1<=m2+m3; s2<=m4+m5; s3<=m6+m7; end
reg [65:0] s4,s5;
always @(posedge clk) begin s4<=s0+s1; s5<=s2+s3; end
reg [66:0] s6;
always @(posedge clk) s6<=s4+s5;
always @(posedge clk) y<=s6+m8;
endmodule