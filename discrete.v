`default_nettype none

`define ALU_OP_ADD 0
`define ALU_OP_OR 1
`define ALU_OP_AND 2
`define ALU_OP_XOR 3
`define ALU_OP_SLT 4
`define ALU_OP_SHIFT 5
`define SHIFT_DIR_LEFT 0
`define SHIFT_DIR_RIGHT 1
`define RD_ALU 0
`define RD_MEM 1
`define RD_PC 2

`ifndef RVFI_OUTPUTS
    `define RVFI_OUTPUTS
`endif

module discrete_core (
    input  wire        i_clk,
    input  wire        i_rst_n,

    `ifdef RISCV_FORMAL
        `RVFI_OUTPUTS,
    `endif

    output wire [31:0] o_imem_addr,
    input  wire [31:0] i_imem_rdata,
    output wire [31:0] o_dmem_addr,
    output wire [31:0] o_dmem_wdata,
    output wire [3:0]  o_dmem_wmask,
    output wire        o_dmem_ren,
    input  wire [31:0] i_dmem_rdata
);
    reg [31:0] pc;
    wire [31:0] next_pc;
    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            pc <= 32'h0;
        else
            pc <= next_pc;
    end

    // if asserted, fetch just emitted a new instruction
    // else, fetch is stalling and effectively buffering the same instruction
    reg if_start;
    wire if_stall;
    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            if_start <= 1'b1;
        else
            if_start <= !if_stall;
        // else if (!if_stall)
        //     if_start <= !if_stall;
    end

    assign o_imem_addr = pc;
    wire [31:0] inst = i_imem_rdata;

    // now that we have {inst, pc, if_start} as the output of the fetch stage,
    // everything through decode and most of execute is combinational
    wire [4:0] rs1_addr, rs2_addr, rd_addr;
    wire rd_wen;
    wire [2:0] rd_sel;
    wire op1_sel, alu_op2_sel, comp_op2_sel;
    wire [5:0] alu_op;
    wire [1:0] shift_dir;
    wire alu_sub, alu_sltu, shift_arith;
    wire jump, branch;
    wire branch_equal, branch_unsigned, branch_invert;
    wire [31:0] imm;
    wire [1:0] mem_width;
    wire mem_ren, mem_wen, mem_unsigned;
    decoder id (
        .i_inst(inst),
        .o_rs1_addr(rs1_addr), .o_rs2_addr(rs2_addr), .o_rd_addr(rd_addr),
        .o_rd_wen(rd_wen),
        .o_rd_sel(rd_sel),
        .o_op1_sel(op1_sel), .o_alu_op2_sel(alu_op2_sel), .o_comp_op2_sel(comp_op2_sel),
        .o_alu_op(alu_op), .o_alu_sub(alu_sub), .o_alu_sltu(alu_sltu),
        .o_shift_dir(shift_dir), .o_shift_arith(shift_arith),
        .o_jump(jump), .o_branch(branch),
        .o_branch_equal(branch_equal),
        .o_branch_unsigned(branch_unsigned),
        .o_branch_invert(branch_invert),
        .o_imm(imm),
        .o_mem_ren(mem_ren), .o_mem_wen(mem_wen),
        .o_mem_width(mem_width), .o_mem_unsigned(mem_unsigned)
    );

    wire [31:0] rs1_rdata, rs2_rdata, rd_wdata;
    rf rf (
        .i_clk(i_clk),
        .i_rs1_addr(rs1_addr), .i_rs2_addr(rs2_addr), .i_rd_addr(rd_addr),
        `ifdef RISCV_FORMAL
        .i_rd_wen(rd_wen && !if_stall && rvfm_valid && !rvfm_trap),
        `else
        .i_rd_wen(rd_wen && !if_stall),
        `endif
        .i_rd_wdata(rd_wdata),
        .o_rs1_rdata(rs1_rdata), .o_rs2_rdata(rs2_rdata)
    );

    wire [31:0] comp_op2 = comp_op2_sel ? imm : rs2_rdata;

    wire eq, lt, ltu;
    comparator comp (
        .i_op1(rs1_rdata), .i_op2(comp_op2),
        .o_eq(eq), .o_lt(lt), .o_ltu(ltu)
    );
    wire take = (branch_equal ? eq : (branch_unsigned ? ltu : lt)) ^ branch_invert;

    wire [31:0] op1 = op1_sel ? pc : rs1_rdata;
    wire [31:0] op2 = alu_op2_sel ? imm : rs2_rdata;
    wire [31:0] alu_result;
    alu alu (
        .i_op(alu_op), .i_sub(alu_sub), .i_sltu(alu_sltu),
        .i_dir(shift_dir), .i_arith(shift_arith),
        .i_op1(op1), .i_op2(op2), .i_lt(lt), .i_ltu(ltu),
        .o_result(alu_result)
    );

    assign o_dmem_addr  = {alu_result[31:2], 2'b00};
    assign o_dmem_ren   = mem_ren;

    wire [1:0] shift_amount = {alu_result[1] && !mem_width[1], alu_result[0] && !mem_width[0]};
    wire [1:0] bshift_dir = {mem_ren, mem_wen};
    wire shift_nop = shift_amount == 2'b00;
    wire [31:0] shift_data;
    wire shift_done;
    wire [31:0] bshift_operand = mem_ren ? i_dmem_rdata : rs2_rdata;
    byte_shifter shifter (
        .i_clk(i_clk), .i_rst_n(i_rst_n),
        .i_operand(bshift_operand), .i_amount(shift_amount),
        .i_start(if_start), .i_dir(bshift_dir), .i_arith(shift_arith),
        .o_result(shift_data), .o_done(shift_done)
    );

    wire mem_word = mem_width[1];
    wire mem_half = mem_width[0];
    wire mem_byte = !(mem_word | mem_half);

    wire half0 = mem_half && !alu_result[1];
    wire half1 = mem_half &&  alu_result[1];
    wire byte0 = mem_byte && !alu_result[1] && !alu_result[0];
    wire byte1 = mem_byte && !alu_result[1] &&  alu_result[0];
    wire byte2 = mem_byte &&  alu_result[1] && !alu_result[0];
    wire byte3 = mem_byte &&  alu_result[1] &&  alu_result[0];

    wire [3:0] wmask;
    assign wmask[3] = mem_wen && (mem_word || half1 || byte3);
    assign wmask[2] = mem_wen && (mem_word || half1 || byte2);
    assign wmask[1] = mem_wen && (mem_word || half0 || byte1);
    assign wmask[0] = mem_wen && (mem_word || half0 || byte0);

    assign o_dmem_wmask = wmask;
    assign o_dmem_wdata = shift_data; // shift_nop ? rs2_rdata : shift_data;

    // TODO: this is only correct if legal memory access boundaries are word
    // aligned, should add proper read masking (this isn't expensive)
    wire [31:0] load_data = shift_data;
    wire [31:0] load_mask = {{16{mem_word}}, {8{mem_word || mem_half}}, 8'hff};
    wire [31:0] load_sign = {32{!mem_unsigned && (mem_half ? load_data[15] : load_data[7])}};
    wire [31:0] load_result = (load_data & load_mask) | (load_sign & ~load_mask);

    // only load/store instructions stall, until the byte shifter is done
    assign if_stall = (mem_wen || mem_ren) && (if_start || !shift_done);

`ifdef RISCV_FORMAL
    wire [31:0] pc_inc = !rvfm_valid ? pc : (pc + 32'h4);
`else
    wire [31:0] pc_inc = pc + 32'h4;
`endif
    wire pc_sel = jump || (branch && take);
    assign next_pc = if_stall ? pc : (pc_sel ? {alu_result[31:1], 1'b0} : pc_inc);

    wire [31:0] rd_sel_alu = {32{rd_sel[`RD_ALU]}};
    wire [31:0] rd_sel_mem = {32{rd_sel[`RD_MEM]}};
    wire [31:0] rd_sel_pc  = {32{rd_sel[`RD_PC]}};
    assign rd_wdata = (alu_result & rd_sel_alu) | (load_result & rd_sel_mem) | (pc_inc & rd_sel_pc);

`ifdef RISCV_FORMAL
// Formal monitor for the discrete core. This is included in the
// core when it's being verified, and disabled for synthesis.

wire [6:0] rvfm_opcode = inst[6:0];
wire [2:0] rvfm_funct3 = inst[14:12];
wire [6:0] rvfm_funct7 = inst[31:25];

// The discrete core is an in-order single cycle core,
// so it "retires" a single instruction every clock cycle. However,
// the core invokes undefined behavior when illegal
// instructions are passed; in order to satisfy the formal interface,
// the formal monitor validates incoming instructions.
reg rvfm_valid;
always @(*) begin
    rvfm_valid = 1'b0;

    if (i_rst_n) begin
        case (rvfm_opcode)
            // lui, auipc, jal
            7'b0110111, 7'b0010111, 7'b1101111: rvfm_valid = 1'b1;
            // jalr
            7'b1100111: rvfm_valid = rvfm_funct3 == 3'b000;
            // beq, bne, blt, bge, bltu, bgeu
            7'b1100011: begin
                casez (rvfm_funct3)
                    3'b00?, 3'b10?, 3'b11?: rvfm_valid = 1'b1;
                endcase
            end
            // lb, lh, lw, lbu, lhu
            7'b0000011: begin
                case (rvfm_funct3)
                    3'b000, 3'b001, 3'b010, 3'b100, 3'b101: rvfm_valid = 1'b1;
                endcase
            end
            // sb, sh, sw
            7'b0100011: begin
                case (rvfm_funct3)
                    3'b000, 3'b001, 3'b010: rvfm_valid = 1'b1;
                endcase
            end
            // addi, slti, sltiu, xori, ori, andi, slli, srli, srai
            7'b0010011: begin
                case (rvfm_funct3)
                    // addi, slti, sltiu
                    3'b000, 3'b010, 3'b011: rvfm_valid = 1'b1;
                    // xori, ori, andi
                    3'b100, 3'b110, 3'b111: rvfm_valid = 1'b1;
                    // slli
                    3'b001: rvfm_valid = rvfm_funct7 == 7'b0000000;
                    // srli, srai
                    3'b101: rvfm_valid = rvfm_funct7 == 7'b0000000 || rvfm_funct7 == 7'b0100000;
                endcase
            end
            // add, sub, sll, slt, sltu, xor, srl, sra, or, and
            7'b0110011: begin
                case (rvfm_funct3)
                    // add, sub
                    3'b000: rvfm_valid = rvfm_funct7 == 7'b0000000 || rvfm_funct7 == 7'b0100000;
                    // slt, sltu
                    3'b010, 3'b011: rvfm_valid = rvfm_funct7 == 7'b0000000;
                    // xor, or, and
                    3'b100, 3'b110, 3'b111: rvfm_valid = rvfm_funct7 == 7'b0000000;
                    // sll
                    3'b001: rvfm_valid = rvfm_funct7 == 7'b0000000;
                    // srl, sra
                    3'b101: rvfm_valid = (rvfm_funct7 == 7'b0000000) || (rvfm_funct7 == 7'b0100000);
                endcase
            end
        endcase
    end
end

// In order retire
reg [63:0] rvfm_retire_ctr;
always @(posedge i_clk, negedge i_rst_n) begin
    if (!i_rst_n)
        rvfm_retire_ctr <= 64'h0;
    else if (!if_stall)
        rvfm_retire_ctr <= rvfm_retire_ctr + 64'h1;
end

// trap on invalid instruction, misaligned memory access, misaligned jump
reg rvfm_trap;
always @(*) begin
    rvfm_trap = 1'b0;

    if (!rvfm_valid && i_rst_n) begin
        rvfm_trap = 1'b1;
    end else begin
        case (rvfm_opcode)
            // load misaligned
            7'b0000011: begin
                case (rvfm_funct3)
                    3'b010: rvfm_trap = alu_result[1:0] != 2'b00;
                    3'b001, 3'b101: rvfm_trap = alu_result[0] != 1'b0;
                endcase
            end
            // store misaligned
            7'b0100011: begin
                case (rvfm_funct3)
                    3'b010: rvfm_trap = alu_result[1:0] != 2'b00;
                    3'b001: rvfm_trap = alu_result[0] != 1'b0;
                endcase
            end
            // branch, jal, jalr target misaligned
            7'b1100011, 7'b1101111, 7'b1100111: rvfm_trap = next_pc[1:0] != 2'b00;
        endcase
    end
end

always @(posedge i_clk) begin
    if (!$initstate && ($past(if_stall))) begin
        assume ($stable(inst));
        assume ($stable(i_dmem_rdata));
    end
end

assign rvfi_valid = !if_stall; // 1'b1;
assign rvfi_order = rvfm_retire_ctr;
assign rvfi_insn  = inst;
assign rvfi_trap  = rvfm_trap;
assign rvfi_halt  = 1'b0;
assign rvfi_intr  = 1'b0;
assign rvfi_mode  = 2'd3; // M-mode
assign rvfi_ixl   = 2'd1; // 32 bit - TODO

assign rvfi_rs1_addr = rs1_addr;
assign rvfi_rs2_addr = rs2_addr;
assign rvfi_rs1_rdata = rs1_rdata;
assign rvfi_rs2_rdata = rs2_rdata;
assign rvfi_rd_addr = (rd_wen && !rvfm_trap) ? rd_addr : 5'h0;
assign rvfi_rd_wdata = (rvfi_rd_addr == 5'h0) ? 32'h0 : rd_wdata;

assign rvfi_pc_rdata = pc;
assign rvfi_pc_wdata = next_pc;

assign rvfi_mem_addr = o_dmem_addr;
wire [3:0] rvfm_rmask_w = {4{rvfi_valid}}; // 4'b1111;
wire [3:0] rvfm_rmask_h = alu_result[1] ? (rvfm_rmask_w & 4'b1100) : (rvfm_rmask_w & 4'b0011);
wire [3:0] rvfm_rmask_b = alu_result[0] ? (rvfm_rmask_h & 4'b1010) : (rvfm_rmask_h & 4'b0101);
assign rvfi_mem_rmask = mem_width[1] ? rvfm_rmask_w : (mem_width[0] ? rvfm_rmask_h : rvfm_rmask_b);
assign rvfi_mem_wmask = o_dmem_wmask;
assign rvfi_mem_rdata = i_dmem_rdata;
assign rvfi_mem_wdata = o_dmem_wdata;
`endif
endmodule

module decoder (
    input  wire [31:0] i_inst,
    output wire [4:0]  o_rs1_addr,
    output wire [4:0]  o_rs2_addr,
    output wire [4:0]  o_rd_addr,
    output wire        o_op1_sel,
    output wire        o_alu_op2_sel,
    output wire        o_comp_op2_sel,
    output wire [5:0]  o_alu_op,
    output wire        o_alu_sub,
    output wire        o_alu_sltu,
    output wire [1:0]  o_shift_dir,
    output wire        o_shift_arith,
    output wire        o_rd_wen,
    output wire [2:0]  o_rd_sel,
    output wire        o_jump,
    output wire        o_branch,
    output wire        o_branch_equal,
    output wire        o_branch_unsigned,
    output wire        o_branch_invert,
    output wire [31:0] o_imm,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [1:0]  o_mem_width,
    output wire        o_mem_unsigned
);
    wire [4:0] opcode = i_inst[6:2];
    wire [4:0] rs1 = i_inst[19:15];
    wire [4:0] rs2 = i_inst[24:20];
    wire [4:0] rd = i_inst[11:7];
    wire [2:0] funct3 = i_inst[14:12];
    wire [6:0] funct7 = i_inst[31:25];

    // major opcode selection
    wire op_load     = opcode == 5'b00000;
    wire op_misc_mem = opcode == 5'b00011;
    wire op_op_imm   = opcode == 5'b00100;
    wire op_auipc    = opcode == 5'b00101;
    wire op_store    = opcode == 5'b01000;
    wire op_amo      = opcode == 5'b01011;
    wire op_op       = opcode == 5'b01100;
    wire op_lui      = opcode == 5'b01101;
    wire op_branch   = opcode == 5'b11000;
    wire op_jalr     = opcode == 5'b11001;
    wire op_jal      = opcode == 5'b11011;
    wire op_system   = opcode == 5'b11100;

    wire alu_funct7 = funct7 == 7'b0100000;
    wire alu_add  = funct3 == 3'b000;
    wire alu_sl   = funct3 == 3'b001;
    wire alu_sr   = funct3 == 3'b101;
    wire alu_slt  = funct3 == 3'b010;
    wire alu_sltu = funct3 == 3'b011;
    wire alu_xor  = funct3 == 3'b100;
    wire alu_or   = funct3 == 3'b110;
    wire alu_and  = funct3 == 3'b111;

    assign o_rd_wen = op_load || op_op_imm || op_auipc || op_op || op_lui || op_jalr || op_jal;
    assign o_rd_sel[`RD_ALU] = !o_rd_sel[`RD_MEM] && !o_rd_sel[`RD_PC];
    assign o_rd_sel[`RD_MEM] = op_load;
    assign o_rd_sel[`RD_PC] = op_jal || op_jalr;

    assign o_op1_sel = op_branch || op_auipc || op_jal;
    assign o_alu_op2_sel = op_load || op_op_imm || op_auipc || op_store || op_lui || op_branch || op_jal || op_jalr;
    assign o_comp_op2_sel = op_op_imm;

    wire op_inst = op_op_imm || op_op;
    assign o_alu_op[`ALU_OP_ADD] = (op_inst && alu_add) || op_load || op_auipc || op_store || op_lui || op_branch || op_jalr || op_jal;
    assign o_alu_op[`ALU_OP_OR] = op_inst && alu_or;
    assign o_alu_op[`ALU_OP_AND] = op_inst && alu_and;
    assign o_alu_op[`ALU_OP_XOR] = op_inst && alu_xor;
    assign o_alu_op[`ALU_OP_SLT] = op_inst && (alu_slt || alu_sltu);
    assign o_alu_op[`ALU_OP_SHIFT] = op_inst && (alu_sl || alu_sr);
    assign o_alu_sub = op_op && alu_add && alu_funct7;
    assign o_alu_sltu = alu_sltu;

    assign o_shift_dir[`SHIFT_DIR_LEFT] = alu_sl;
    assign o_shift_dir[`SHIFT_DIR_RIGHT] = alu_sr;
    assign o_shift_arith = alu_funct7 || op_store;

    assign o_jump = op_jal || op_jalr;
    assign o_branch = op_branch;
    assign o_branch_equal = !funct3[2];
    assign o_branch_unsigned = funct3[1];
    assign o_branch_invert = funct3[0];

    // immediate decoding
    wire format_r = op_op || op_amo;
    wire format_i = op_op_imm || op_jalr || op_load;
    wire format_s = op_store;
    wire format_b = op_branch;
    wire format_u = op_lui || op_auipc;
    wire format_j = op_jal;

    wire format_sb = format_s | format_b;
    wire format_ij = format_i | format_j;
    wire format_uj = format_u | format_j;

    assign o_imm[0] = (format_s & i_inst[7]) | (format_i && i_inst[20]);
    assign o_imm[4:1] = ({4{format_sb}} & i_inst[11:8]) | ({4{format_ij}} & i_inst[24:21]);
    assign o_imm[10:5] = {6{!format_u}} & i_inst[30:25];
    assign o_imm[11] = format_b ? i_inst[7] : (format_j ? i_inst[20] : (format_u ? 1'b0 : i_inst[31]));
    assign o_imm[19:12] = format_uj ? i_inst[19:12] : {8{i_inst[31]}};
    assign o_imm[30:20] = format_u ? i_inst[30:20] : {11{i_inst[31]}};
    assign o_imm[31] = i_inst[31];

    assign o_rs1_addr = rs1 & {5{!op_lui}};
    assign o_rs2_addr = rs2;
    assign o_rd_addr = rd;

    assign o_mem_ren = op_load;
    assign o_mem_wen = op_store;
    assign o_mem_width = funct3[1:0];
    assign o_mem_unsigned = funct3[2];

`ifdef DISCRETE_FORMAL
    always @(*) begin
        assert ((op_load + op_misc_mem + op_op_imm + op_auipc + op_store + op_amo + op_op + op_lui + op_branch + op_jalr + op_jal + op_system) == 1);
        assert ((o_alu_op[0] + o_alu_op[1] + o_alu_op[2] + o_alu_op[3] + o_alu_op[4]) == 1);
        if (!o_alu_op[`ALU_OP_ADD])
            assert (!o_alu_sub);
        if (o_alu_op[`ALU_OP_SHIFT])
            assert ((o_shift_dir[0] + o_shift_dir[1]) == 1);
        assert ((o_rd_sel[0] + o_rd_sel[1] + o_rd_sel[2]) == 1);
    end
`endif
endmodule

module sram (
    input  wire [4:0] i_addr,
    input  wire       i_wen,
    input  wire [7:0] i_data,
    output wire [7:0] o_data
);
    // reg [7:0] mem [31:0];
    // always @(*) begin
    //     if (i_wen)
    //         mem[i_addr] <= i_data;
    // end
    //
    // assign o_data = mem[i_addr];
endmodule

module rf (
    input  wire        i_clk,
    input  wire [4:0]  i_rs1_addr,
    input  wire [4:0]  i_rs2_addr,
    input  wire [4:0]  i_rd_addr,
    input  wire        i_rd_wen,
    input  wire [31:0] i_rd_wdata,
    output wire [31:0] o_rs1_rdata,
    output wire [31:0] o_rs2_rdata
);
    wire dir = i_clk;
    // on posedge, write data to both RAMs if write enable asserted
    // on negedge, read independently from both RAMs
    wire [4:0] addr0 = dir ? i_rd_addr : i_rs1_addr;
    wire [4:0] addr1 = dir ? i_rd_addr : i_rs2_addr;

    wire wen = dir && i_rd_wen;
    wire [31:0] rs1_zero = {32{!(|i_rs1_addr)}};
    wire [31:0] rs2_zero = {32{!(|i_rs2_addr)}};

    wire [31:0] rs1_rdata, rs2_rdata;
`ifdef RISCV_FORMAL
    (* keep *) reg [31:0] file [0:31];
    assign rs1_rdata = file[i_rs1_addr];
    assign rs2_rdata = file[i_rs2_addr];
    always @(posedge i_clk)
        if (i_rd_wen)
            file[i_rd_addr] <= i_rd_wdata;
`else
    // sram bank0 [3:0] (.i_addr(addr0), .i_wen(wen), .i_data(i_rd_wdata), .o_data(rs1_rdata));
    // sram bank1 [3:0] (.i_addr(addr1), .i_wen(wen), .i_data(i_rd_wdata), .o_data(rs2_rdata));
`endif

    // on read, the RAMs will drive the data lines with read data
    // no need to gate this for read, but we mask so that reads to
    // x0 output zero despite the underlying write succeeding
    assign o_rs1_rdata = rs1_rdata & ~rs1_zero;
    assign o_rs2_rdata = rs2_rdata & ~rs2_zero;

`ifdef DISCRETE_FORMAL
    always @(*) begin
        if (i_rs1_addr == 0)
            assert (o_rs1_rdata == 32'h0);
        if (i_rs2_addr == 0)
            assert (o_rs2_rdata == 32'h0);
    end
`endif
endmodule

module alu (
    input  wire [5:0]  i_op,
    // assert to enable subtraction mode on the adder
    input  wire [31:0] i_op1,
    input  wire [31:0] i_op2,
    input  wire        i_lt,
    input  wire        i_ltu,
    input  wire        i_sub,
    input  wire        i_sltu,
    input  wire [1:0]  i_dir,
    input  wire        i_arith,
    output wire [31:0] o_result
);
    wire op_or  = i_op[`ALU_OP_OR];
    wire op_and = i_op[`ALU_OP_AND];
    wire op_xor = i_op[`ALU_OP_XOR];
    wire [1:0] bool_op = {op_and | op_or | i_op[`ALU_OP_ADD], op_xor | op_or};

    wire [31:0] i_b = i_op2 ^ {32{i_sub}};
    wire [31:0] xor_result = i_op1 ^ i_b;
    wire [31:0] and_result = i_op1 & i_b;
    wire [31:0] cin;
    wire [31:0] bool_cin = {32{bool_op[0]}} | (cin & {32{i_op[`ALU_OP_ADD]}});
    wire [31:0] bool_result = (xor_result & bool_cin) | (and_result & {32{bool_op[1]}});
    wire [31:0] add_result = xor_result ^ cin;
    wire [31:0] cout = bool_result;
    assign cin = {cout[30:0], i_sub};

    wire slt_result = i_sltu ? i_ltu : i_lt;
    wire [31:0] shift_result;
    shifter shifter (
        .i_op1(i_op1), .i_op2(i_op2[4:0]),
        .i_dir(i_dir), .i_arith(i_arith),
        .o_result(shift_result)
    );

    wire [30:0] op_add   = {32{i_op[`ALU_OP_ADD]}};
    wire [30:0] op_bool  = {32{op_or | op_and | op_xor}};
    wire [30:0] op_shift = {32{i_op[`ALU_OP_SHIFT]}};
    wire        op_slt   = i_op[`ALU_OP_SLT];
    assign o_result[0] = (add_result[0] & op_add[0]) | (bool_result[0] & op_bool[0]) | (slt_result & op_slt) | (shift_result[0] & op_shift[0]);
    assign o_result[31:1] = (add_result[31:1] & op_add) | (bool_result[31:1] & op_bool) | (shift_result[31:1] & op_shift);

`ifdef DISCRETE_FORMAL
    always @(*) begin
        assume ((i_op[0] + i_op[1] + i_op[2] + i_op[3] + i_op[4]) == 1);
        if (!i_op[0])
            assume (!i_sub);

        case (1'b1)
            i_op[`ALU_OP_ADD]: assert (o_result == (i_sub ? (i_op1 - i_op2) : (i_op1 + i_op2)));
            i_op[`ALU_OP_OR]:  assert (o_result == (i_op1 | i_op2));
            i_op[`ALU_OP_AND]: assert (o_result == (i_op1 & i_op2));
            i_op[`ALU_OP_XOR]: assert (o_result == (i_op1 ^ i_op2));
            i_op[`ALU_OP_SLT]: assert (o_result == ({31'h0, i_sltu ? i_ltu : i_lt}));
        endcase
    end
`endif
endmodule

module shifter (
    input  wire [31:0] i_op1,
    input  wire [4:0]  i_op2,
    input  wire [1:0]  i_dir, // bit 0 = left, bit 1 = right
    input  wire        i_arith,
    output wire [31:0] o_result
);
    wire [31:0] sl4 = i_op2[4] ? {i_op1[15:0], 16'h0000} : i_op1;
    wire [31:0] sl3 = i_op2[3] ? {sl4[23:0], 8'h00} : sl4;
    wire [31:0] sl2 = i_op2[2] ? {sl3[27:0], 4'h0} : sl3;
    wire [31:0] sl1 = i_op2[1] ? {sl2[29:0], 2'b00} : sl2;
    wire [31:0] sl0 = i_op2[0] ? {sl1[30:0], 1'b0} : sl1;

    wire sign = i_arith && i_op1[31];
    wire [31:0] sr4 = i_op2[4] ? {{16{sign}}, i_op1[31:16]} : i_op1;
    wire [31:0] sr3 = i_op2[3] ? {{8{sign}}, sr4[31:8]} : sr4;
    wire [31:0] sr2 = i_op2[2] ? {{4{sign}}, sr3[31:4]} : sr3;
    wire [31:0] sr1 = i_op2[1] ? {{2{sign}}, sr2[31:2]} : sr2;
    wire [31:0] sr0 = i_op2[0] ? {sign, sr1[31:1]} : sr1;

    wire [31:0] op_sl = {32{i_dir[0]}};
    wire [31:0] op_sr = {32{i_dir[1]}};
    assign o_result = (op_sl & sl0) | (op_sr & sr0);

`ifdef DISCRETE_FORMAL
    always @(*) begin
        assume ((i_dir[0] + i_dir[1]) == 1);
        if (i_dir[0])
            assert (o_result == (i_op1 << i_op2[4:0]));
        else if (i_arith)
            assert ($signed(o_result) == ($signed(i_op1) >>> i_op2[4:0]));
        else
            assert (o_result == (i_op1 >> i_op2[4:0]));
    end
`endif
endmodule

module comparator (
    input  wire [31:0] i_op1,
    input  wire [31:0] i_op2,
    output wire        o_eq,
    output wire        o_lt,
    output wire        o_ltu
);
    // equality
    assign o_eq = &(i_op1 ~^ i_op2);

    // less than
    wire [31:0] ltu_bits;
    assign ltu_bits = (~i_op1 & i_op2) | ((i_op1 ~^ i_op2) & {ltu_bits[30:0], 1'b0});
    assign o_lt = (i_op1[31] == i_op2[31]) ? ltu_bits[30] : i_op1[31];
    assign o_ltu = ltu_bits[31];

`ifdef DISCRETE_FORMAL
    always @(*) begin
        assert (o_eq == (i_op1 == i_op2));
        assert (o_lt == ($signed(i_op1) < $signed(i_op2)));
        assert (o_ltu == (i_op1 < i_op2));
    end
`endif
endmodule

module byte_shifter (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire [31:0] i_operand,
    input  wire [1:0]  i_amount,
    input  wire        i_start,
    input  wire [1:0]  i_dir, // bit 0 = left, bit 1 = right
    input  wire        i_arith,
    output wire [31:0] o_result,
    output wire        o_done
);
    reg [1:0] amount;
    reg [31:0] result;
    reg done;

    wire [1:0] next_amount = i_start ? i_amount : (amount - 2'd1);
    wire sign = i_arith && i_operand[31];
    wire [31:0] shift_result = ({32{i_dir[0]}} & {result[23:0], 8'h00}) | ({32{i_dir[1]}} & {{8{sign}}, result[31:8]});
    wire [31:0] next_result = i_start ? i_operand : shift_result;
    wire next_done = next_amount == 2'd0;
    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n) begin
            amount <= 2'd0;
            result <= 32'd0;
            done <= 1'b0;
        end else begin
            amount <= next_amount;
            result <= next_result;
            done <= next_done;
        end
    end

    assign o_result = result;
    assign o_done = done;
endmodule
