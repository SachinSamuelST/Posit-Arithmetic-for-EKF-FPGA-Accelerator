`timescale 1ns / 1ps

module tb_posit_mult();

    reg  [15:0] a, b;
    wire [15:0] out;

    posit_mult #(.N(16), .ES(2)) uut (
        .a(a), .b(b), .out(out)
    );

    initial begin
        $display("--------------------------------------------------");
        $display("   Posit(16, 2) Multiplier Testbench Verification ");
        $display("--------------------------------------------------");

        // Test Case 1: 1.0 * 1.0 = 1.0
        a = 16'h4000; b = 16'h4000; #10;
        $display("TC1: 1.0 * 1.0      | In: %h * %h | Out: %h (Expected: 4000)", a, b, out);

        // Test Case 2: 1.5 * 2.0 = 3.0
        a = 16'h4400; b = 16'h4800; #10;
        $display("TC2: 1.5 * 2.0      | In: %h * %h | Out: %h (Expected: 4c00)", a, b, out);

        // Test Case 3: 2.0 * 2.0 = 4.0
        a = 16'h4800; b = 16'h4800; #10;
        $display("TC3: 2.0 * 2.0      | In: %h * %h | Out: %h (Expected: 5000)", a, b, out);

        // Test Case 4: 0.5 * 2.0 = 1.0
        a = 16'h3800; b = 16'h4800; #10;
        $display("TC4: 0.5 * 2.0      | In: %h * %h | Out: %h (Expected: 4000)", a, b, out);

        // Test Case 5: 1.5 * -2.0 = -3.0
        a = 16'h4400; b = 16'hB800; #10;
        $display("TC5: 1.5 * -2.0     | In: %h * %h | Out: %h (Expected: b400)", a, b, out);

        $display("--------------------------------------------------");
        $finish;
    end
endmodule