`default_nettype none

module shifter (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire [31:0] i_op1,
    input  wire [4:0]  i_op2,
    input  wire [1:0]  i_dir,
    input  wire        i_start,
    output wire [31:0] o_result,
    output wire        o_ready
);
    // wire [2:0] rough_count = i_op2[4:2];
    // wire [1:0] fine_count = i_op2[1:0];
    reg [2:0] rough;
    reg [1:0] fine;

    reg [2:0] state;
    // state transitions:
    // i_start: idle -> rough
    // (rough == 0): rough -> fine
    // (fine == 0): fine -> idle
    wire next_rough = state[2] && i_start;
    wire next_fine = state[1] && (rough == 3'd0);
    wire next_idle = state[0] && (fine == 3'd0);
    wire [2:0] next_state = {next_idle, next_rough, next_idle};

    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= 3'b100;
            rough <= 3'd0;
            fine <= 2'd0;
        end else begin
            state <= next_state;
            rough <= next_rough;
            fine <= next_fine;
        end
    end
endmodule
