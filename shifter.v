`default_nettype none

module shifter (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire [31:0] i_op1,
    input  wire [4:0]  i_op2,
    // input  wire [1:0]  i_dir,
    input  wire        i_start,
    output wire [31:0] o_result,
    output wire        o_done
);
    reg [4:0] amount, next_amount;
    always @(*) begin
        next_amount = amount;
        if (i_start)
            next_amount = i_op2;
        else if (amount != 5'd0)
            next_amount = amount - 5'd1;
    end

    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            amount <= 5'd0;
        else
            amount <= next_amount;
    end

    reg [31:0] result, next_result;
    always @(*) begin
        next_result = result;
        if (i_start)
            next_result = i_op1;
        else if (amount != 5'd0)
            next_result = {next_result[30:0], 1'b0};
    end

    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            result <= 32'd0;
        else
            result <= next_result;
    end

    reg done;
    always @(posedge i_clk, negedge i_rst_n) begin
        if (!i_rst_n)
            done <= 1'b0;
        else
            done <= (i_start || (amount != 5'd0)) && (next_amount == 5'd0);
    end

    assign o_result = result;
    assign o_done = done;

`ifdef DISCRETE_FORMAL
    reg f_past_valid;
    initial f_past_valid <= 1'b0;
    always @(posedge i_clk)
        f_past_valid <= 1'b1;

    initial assume (!i_rst_n);
    initial assume (amount == 0);

    reg [31:0] f_operand;
    reg [4:0] f_amount;
    reg [4:0] f_counter;
    always @(*) begin
        if (i_start) begin
            f_operand <= i_op1;
            f_amount <= i_op2;
        end
    end

    reg f_busy;
    initial f_busy <= 1'b0;
    always @(posedge i_clk) begin
        if (i_rst_n) begin
            if (f_past_valid && $past(i_start))
                assume (!i_start);

            if (i_start)
                f_busy <= i_op2 != 0;

            if (f_past_valid && $rose(o_done)) begin
                f_busy <= 1'b0;
                assert (o_result == (f_operand << f_amount));
            end

            if (f_busy)
                assume (!i_start);

            if (f_past_valid && o_done) begin
                cover ((o_result != 0) && o_result == $past(i_op1));
                cover ((o_result != 0) && o_result != $past(i_op1));
            end
        end
    end
`endif
endmodule
