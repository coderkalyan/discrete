`default_nettype none

// `define DISCRETE_FORMAL

module core ();
    alu alu (.i_op(), .i_op1(), .i_op2(), .i_sub(), .o_result());
    comparator comp (.i_op1(), .i_op2());
    shifter shifter (.i_op1(), .i_op2(), .o_result());
endmodule

// This is a module of an external 32K 8bit-word SRAM. It's external to the
// design and therefore not optimized, just a cycle accurate model.
// TODO: this is a fake interface, the actual chip is level triggered
// module sram_32k8 (
//     input  wire        i_clk,
//     input  wire        i_rst_n,
//     input  wire [14:0] i_addr,
//     input  wire [7:0]  i_data,
//     input  wire        i_wen,
//     output wire [7:0]  o_data
// );
//     reg [7:0] mem [14:0];
//     always @(posedge i_clk, negedge i_rst_n) begin
//         if (!i_rst_n) begin
//             for (integer i = 0; i < 32768; i = i + 1) mem[i] <= 8'h00;
//         end else begin
//             if (i_wen) mem[i_addr] <= i_data;
//         end
//     end
//
//     assign o_data = mem[i_addr];
// endmodule

// module rf (
//     input  wire        i_clk,
//     input  wire        i_rst_n,
//     input  wire [4:0]  i_rs1_addr,
//     input  wire [4:0]  i_rs2_addr,
//     input  wire [4:0]  i_rd_addr,
//     input  wire        i_rd_wen,
//     input  wire [31:0] i_rd_wdata,
//     output wire [31:0] o_rs1_data,
//     output wire [31:0] o_rs2_data
// );
//     sram_32k8 bank0 [3:0] (
//         .i_clk(i_clk), .i_rst_n(i_rst_n),
//         .i_addr({10'h0, i_rs1_addr}),
//         .i_
//         .o_data(o_rs1_data)
//     );
//     sram_32k8 bank1 [3:0] (
//         .i_clk(i_clk), .i_rst_n(i_rst_n),
//         .i_addr()
//     );
// endmodule

// 32 bit ripple carry adder
module rca (
    input  wire [31:0] i_a,
    input  wire [31:0] i_b,
    input  wire        i_cin,
    output wire [31:0] o_sum,
    output wire [31:0] o_xor,
    output wire [31:0] o_and
);
    wire [31:0] cin, cout;

    assign cin[0] = i_cin;
    assign o_xor = i_a ^ i_b;
    assign o_sum = o_xor ^ cin;
    assign o_and = i_a & i_b;
    assign cout = o_and | (i_a & cin) | (i_b & cin);
    assign cin[31:1] = cout[30:0];
endmodule

module alu (
    // 0 = add
    // 1 = or
    // 2 = and
    // 3 = xor
    input  wire [3:0]  i_op,
    // assert to enable subtraction mode on the adder
    input  wire [31:0] i_op1,
    input  wire [31:0] i_op2,
    input  wire        i_sub,
    output wire [31:0] o_result
);
    wire [31:0] or_result = i_op1 | i_op2;
    wire [31:0] add_result, xor_result, and_result;
    rca adder (
        .i_a(i_op1), .i_b(i_op2 ^ {32{i_sub}}), .i_cin(i_sub),
        .o_sum(add_result), .o_xor(xor_result), .o_and(and_result)
    );

    wire [31:0] op_add = {32{i_op[0]}};
    wire [31:0] op_or  = {32{i_op[1]}};
    wire [31:0] op_and = {32{i_op[2]}};
    wire [31:0] op_xor = {32{i_op[3]}};
    assign o_result = (add_result & op_add) | (or_result & op_or) | (and_result & op_and) | (xor_result & op_xor);

`ifdef DISCRETE_FORMAL
    always @(*) begin
        assume ((i_op[0] + i_op[1] + i_op[2] + i_op[3]) == 1);
        if (!i_op[0])
            assume (!i_sub);

        case (1'b1)
            i_op[0]: assert (o_result == (i_sub ? (i_op1 - i_op2) : (i_op1 + i_op2)));
            i_op[1]: assert (o_result == (i_op1 | i_op2));
            i_op[2]: assert (o_result == (i_op1 & i_op2));
            i_op[3]: assert (o_result == (i_op1 ^ i_op2));
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
    output wire        o_ne,
    output wire        o_lt,
    output wire        o_ltu,
    output wire        o_ge,
    output wire        o_geu
);
    // equality
    wire o_eq = &(i_op1 ~^ i_op2);
    wire o_ne = !o_eq;

    // less than
    wire [31:0] ltu_bits;
    assign ltu_bits = (~i_op1 & i_op2) | ((i_op1 ~^ i_op2) & {ltu_bits[30:0], 1'b0});
    wire o_lt = (i_op1[31] == i_op2[31]) ? ltu_bits[30] : i_op1[31];
    wire o_ltu = ltu_bits[31];

    // greater than equal
    wire o_ge = !o_lt | o_eq;
    wire o_geu = !o_ltu | o_eq;

`ifdef DISCRETE_FORMAL
    always @(*) begin
        assert (o_eq == (i_op1 == i_op2));
        assert (o_ne == (i_op1 != i_op2));
        assert (o_ltu == (i_op1 < i_op2));
        assert (o_lt == ($signed(i_op1) < $signed(i_op2)));
        assert (o_geu == (i_op1 >= i_op2));
        assert (o_ge == ($signed(i_op1) >= $signed(i_op2)));
    end
`endif
endmodule

module lsu (
    input  wire [31:0] i_wdata,
    output wire [31:0] o_mem_wdata,
    output wire [3:0]  o_mem_wmask,
    input  wire [31:0] i_mem_rdata,
    output wire [31:0] o_rdata
);
endmodule
