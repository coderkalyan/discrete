// Formal monitor for the discrete core. This is included in the
// core when it's being verified, and disabled for synthesis.

wire [6:0] opcode = inst[6:0];
wire [2:0] funct3 = inst[14:12];
wire [6:0] funct7 = inst[31:25];

// The discrete core is an in-order single cycle core,
// so it "retires" a single instruction every clock cycle. However,
// the core invokes undefined behavior when illegal
// instructions are passed; in order to satisfy the formal interface,
// the formal monitor validates incoming instructions.
reg rvfm_valid;
always @(*) begin
    rvfm_valid = 1'b0;

    case (opcode)
        // lui, auipc, jal
        7'b0110111, 7'b0010111, 7'b1101111: rvfm_valid = 1'b1;
        // jalr
        7'b1100111: rvfm_valid = funct3 == 3'b000;
        // beq, bne, blt, bge, bltu, bgeu
        7'b1100011: begin
            casez (funct3)
                3'b00?, 3'b10?, 3'b11?: rvfm_valid = 1'b1;
            endcase
        end
        // lb, lh, lw, lbu, lhu
        7'b0000011: begin
            case (funct3)
                3'b000, 3'b001, 3'b010, 3'b100, 3'b101: rvfm_valid = 1'b1;
            endcase
        end
        // sb, sh, sw
        7'b0100011: begin
            case (funct3)
                3'b000, 3'b001', 3'b010: rvfm_valid = 1'b1;
            endcase
        end
        // addi, slti, sltiu, xori, ori, andi, slli, srli, srai
        7'b0010011: begin
            case (funct3)
                // addi, slti, sltiu
                3'b000, 3'b010, 3'b011: rvfm_valid = 1'b1;
                // xori, ori, andi
                3'b100, 3'b110, 3'b111: rvfm_valid = 1'b1;
                // slli
                3'b001: rvfm_valid = funct7 == 7'b0000000;
                // srli, srai
                3'b101: rvfm_valid = funct7 ==? 7'b0?00000;
            endcase
        end
        // add, sub, sll, slt, sltu, xor, srl, sra, or, and
        7'b0110011: begin
            case (funct3)
                // add, sub
                3'b000: rvfm_valid = funct7 ==? 7'b0?00000;
                // slt, sltu
                3'b010, 3'b011: rvfm_valid = funct7 == 7'b0000000;
                // xor, or, and
                3'b100, 3'b110, 3'b111: rvfm_valid = funct7 == 7'b0000000;
                // sll
                3'b001: rvfm_valid = funct7 == 7'b0000000;
                // srl, sra
                3'b101: rvfm_valid = funct7 ==? 7'b0?00000;
            endcase
        end
    endcase
end

// In order retire
reg [63:0] rvfm_retire_ctr;
always @(posedge i_clk, negedge i_rst_n) begin
    if (!i_rst_n)
        rvfm_retire_ctr <= 64'h0;
    else if (rvfi_valid)
        rvfm_retire_ctr <= rvfm_retire_ctr + 64'h1;
end

// trap on invalid instruction, misaligned memory access, misaligned jump
reg rvfm_trap;
always @(*) begin
    rvfm_trap = 1'b0;

    if (!rvfm_valid) begin
        rvfm_trap = 1'b1;
    end else begin
        case (opcode)
            // load misaligned
            7'b0000011: begin
                case (funct3)
                    3'b010: rvfm_trap = alu_result[1:0] != 2'b00;
                    3'b001, 3'b101: rvfm_trap = alu_result[0] != 1'b0;
                endcase
            end
            // store misaligned
            7'b0100011: begin
                case (funct3)
                    3'b010: rvfm_trap = alu_result[1:0] != 2'b00;
                    3'b001: rvfm_trap = alu_result[0] != 1'b0;
                endcase
            end
            // branch target misaligned
            7'b1100011: rvfm_trap = imm[1:0] != 2'b00;
            // jal target misaligned
            7'b1101111: rvfm_trap = imm[1:0] != 2'b00;
            // jalr target misaligned
            7'b1100111: rvfm_trap = alu_result[2:0] != 2'b00;
        endcase
    end
end

assign rvfi_valid = rvfm_valid;
assign rvfi_order = rvfm_retire_ctr;
assign rvfi_insn  = inst;
assign rvfi_trap  = rvfm_trap;
assign rvfi_halt  = 1'b0;
assign rvfi_intr  = 1'b0;
assign rvfi_mode  = 2'b11; // M-mode
assign rvfi_ixl   = 2'b01;  // 32 bit - TODO

assign rvfi_rs1_addr = rs1_addr;
assign rvfi_rs2_addr = rs2_addr;
assign rvfi_rs1_rdata = rs1_rdata;
assign rvfi_rs2_rdata = rs2_rdata;
assign rvfi_rd_addr = rd_wen ? rd_addr : 5'h0;
assign rvfi_rd_wdata = (rvfi_rd_addr == 5'h0) ? 32'h0 : rd_wdata;

assign pc_rdata = pc;
assign pc_wdata = next_pc;

assign rvfi_mem_addr = 32'hx;
assign rvfi_mem_rmask = 4'h0;
assign rvfi_mem_wmask = 4'h0;
assign rvfi_mem_rdata = 32'h0;
assign rvfi_mem_wdata = 32'h0;
