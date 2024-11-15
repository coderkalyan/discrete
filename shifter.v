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
    reg [2:0]  rough;
    reg [1:0]  fine;
    reg [31:0] result;
    reg        ready;

    // one hot state machine with three states:
    // * idle
    // * rough (shift by 4 each time)
    // * fine (shift by 1 each time)
    reg [2:0] state;
    wire [2:0] next_state;
    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            state <= 3'b100;
        else
            state <= next_state;
    end

    // idle -> rough on i_start
    wire ns_rough = state[2] && i_start;
    // rough -> fine on rough == 0
    wire ns_fine  = state[1] && (rough == 3'd0);
    // fine -> idle on fine == 0
    wire ns_idle  = state[0] && (fine == 3'd0);
    wire rough_zero = rough == 3'd0;
    wire fine_zero = fine == 2'd0;
    assign next_state[2] = (state[2] && !i_start)    || (state[0] && fine_zero);
    assign next_state[1] = (state[1] && !rough_zero) || (state[2] && i_start);
    assign next_state[0] = (state[0] && !fine_zero)  || (state[1] && rough_zero);

    // the result scratch register gets latched on start, then shifted
    wire [31:0] sl_rough    = {result[27:0], 4'b0000};
    wire [31:0] sl_fine     = {result[30:0], 1'b0};
    wire [31:0] next_result = ({32{state[2]}} & i_op1) | ({32{state[1]}} & sl_rough) | ({32{state[0]}} & sl_fine);
    // the rough count gets latched on start, then decremented
    wire [2:0] next_rough = ({3{state[2]}} & i_op2[4:2]) | ({3{state[1]}} & (rough - 3'd1));
    // similar with fine count
    wire [1:0] next_fine  = ({2{state[2]}} & i_op2[1:0]) | ({2{state[0]}} & (fine - 3'd1));
    // on the fine -> idle transition, result is correct, assert ready
    wire next_ready = ns_idle;

    always @(posedge i_clk) begin
        result <= next_result;
        rough  <= next_rough;
        fine   <= next_fine;
    end

    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            ready <= 1'b0;
        else
            ready <= next_ready;
    end

    assign o_result = result;
    assign o_ready  = ready;

`ifdef DISCRETE_FORMAL
    reg f_past_valid;
    initial f_past_valid <= 1'b0;
    always @(posedge i_clk)
        f_past_valid <= 1'b1;

    reg [31:0] operand;
    reg [4:0]  shamt;
    always @(posedge i_clk) begin
        assert ((state[0] + state[1] + state[2]) == 1);

        if (i_start) begin
            operand <= i_op1;
            shamt   <= i_op2;
        end

        if (o_ready) begin
            assert (o_result == (operand << shamt));
            assert (!$past(o_ready));
        end
    end
`endif
endmodule
