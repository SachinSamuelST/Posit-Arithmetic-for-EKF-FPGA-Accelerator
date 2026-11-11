`timescale 1ns / 1ps

module tb_posit_div();

    reg  [15:0] a, b;
    wire [15:0] out;

    posit_div #(.N(16), .ES(2)) uut (
        .a(a), .b(b), .out(out)
    );

    initial begin
        $display("--------------------------------------------------");
        $display("     Posit(16, 2) Divider Testbench Verification   ");
        $display("--------------------------------------------------");

        // Test Case 1: 1.0 / 1.0 = 1.0
        a = 16'h4000; b = 16'h4000; #10;
        $display("TC1: 1.0 / 1.0      | In: %h / %h | Out: %h (Expected: 4000)", a, b, out);

        // Test Case 2: 4.0 / 2.0 = 2.0
        a = 16'h5000; b = 16'h4800; #10;
        $display("TC2: 4.0 / 2.0      | In: %h / %h | Out: %h (Expected: 4800)", a, b, out);

        // Test Case 3: 3.0 / 1.5 = 2.0
        a = 16'h4C00; b = 16'h4400; #10;
        $display("TC3: 3.0 / 1.5      | In: %h / %h | Out: %h (Expected: 4800)", a, b, out);

        // Test Case 4: 1.0 / 2.0 = 0.5
        a = 16'h4000; b = 16'h4800; #10;
        $display("TC4: 1.0 / 2.0      | In: %h / %h | Out: %h (Expected: 3800)", a, b, out);

        // Test Case 5: -3.0 / 1.5 = -2.0
        a = 16'hB400; b = 16'h4400; #10;
        $display("TC5: -3.0 / 1.5     | In: %h / %h | Out: %h (Expected: b800)", a, b, out);

        // Test Case 6: 1.0 / 0.0 = NaR
        a = 16'h4000; b = 16'h0000; #10;
        $display("TC6: 1.0 / 0.0      | In: %h / %h | Out: %h (Expected: 8000)", a, b, out);

        $display("--------------------------------------------------");
        $finish;
    end
endmodule