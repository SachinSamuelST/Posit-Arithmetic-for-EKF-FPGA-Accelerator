`timescale 1ns / 1ps

module posit_alu #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    input  wire [N-1:0] a,
    input  wire [N-1:0] b,
    input  wire [1:0]   op, // 00: ADD, 01: SUB, 10: MUL, 11: DIV
    output wire [N-1:0] out,
    output wire         done
);

    wire [N-1:0] add_sub_out;
    wire [N-1:0] mult_out;
    wire [N-1:0] div_out;
    wire         div_done;

    // Instantiate Adder / Subtractor (op[0] controls subtraction)
    posit_add #(.N(N), .ES(ES)) adder_inst (
        .a(a),
        .b(b),
        .sub(op[0]),
        .out(add_sub_out)
    );

    // Instantiate Multiplier
    posit_mult #(.N(N), .ES(ES)) mult_inst (
        .a(a),
        .b(b),
        .out(mult_out)
    );

    // Trigger sequential divider start whenever start pulse is asserted during DIV
    wire div_start = (op == 2'b11) ? start : 1'b0;

    // Instantiate Multi-Cycle Sequential Divider
    posit_div #(.N(N), .ES(ES)) div_inst (
        .clk(clk),
        .rst(rst),
        .start(div_start),
        .a(a),
        .b(b),
        .out(div_out),
        .done(div_done)
    );

    // Combinational Output Mux
    reg [N-1:0] comb_out;
    always @(*) begin
        case (op)
            2'b00:   comb_out = add_sub_out; // ADD
            2'b01:   comb_out = add_sub_out; // SUB
            2'b10:   comb_out = mult_out;    // MUL
            2'b11:   comb_out = div_out;     // DIV
            default: comb_out = {N{1'b0}};
        endcase
    end

    // Direct combinational connection to eliminate 1-cycle pipeline lag
    assign out  = comb_out;
    assign done = (op == 2'b11) ? div_done : 1'b1;

endmodule