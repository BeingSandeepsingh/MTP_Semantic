`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.04.2026 15:26:27
// Design Name: 
// Module Name: CNN_TOP
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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
              (load_cnt << 2);

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
