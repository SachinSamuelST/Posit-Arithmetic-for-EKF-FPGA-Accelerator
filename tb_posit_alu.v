`timescale 1ns / 1ps

module tb_posit_alu();

    reg  [15:0] a, b;
    reg  [1:0]  op;
    wire [15:0] out;

    posit_alu #(.N(16), .ES(2)) uut (
        .a(a), .b(b), .op(op), .out(out)
    );

    initial begin
        $display("==================================================");
        $display("     Posit(16, 2) Top-Level ALU Verification      ");
        $display("==================================================");

        // ADD: 16.0 + 1.0 = 17.0
        op = 2'b00; a = 16'h6000; b = 16'h4000; #10;
        $display("OP 00 (ADD): 16.0 + 1.0  | Out: %h (Expected: 6040)", out);

        // SUB: 16.0 - 1.0 = 15.0
        op = 2'b01; a = 16'h6000; b = 16'h4000; #10;
        $display("OP 01 (SUB): 16.0 - 1.0  | Out: %h (Expected: 5f00)", out);

        // MUL: 1.5 * 2.0 = 3.0
        op = 2'b10; a = 16'h4400; b = 16'h4800; #10;
        $display("OP 10 (MUL): 1.5 * 2.0   | Out: %h (Expected: 4c00)", out);

        // DIV: 4.0 / 2.0 = 2.0
        op = 2'b11; a = 16'h5000; b = 16'h4800; #10;
        $display("OP 11 (DIV): 4.0 / 2.0   | Out: %h (Expected: 4800)", out);

        $display("==================================================");
        $finish;
    end
endmodule