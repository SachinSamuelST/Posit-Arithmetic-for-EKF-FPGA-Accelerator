`timescale 1ns / 1ps

module tb_posit_add();

    reg  [15:0] a, b;
    reg         sub;
    wire [15:0] out;

    posit_add #(.N(16), .ES(2)) uut (
        .a(a), .b(b), .sub(sub), .out(out)
    );

    initial begin
        $display("=========================================================");
        $display(" Time  |   A    |   B    | SUB |  OUT   | Operation");
        $display("=========================================================");

        // Test Case 1: 1.0 + 1.0 = 2.0 (0x4800)
        a = 16'h4000; b = 16'h4000; sub = 0; #10;
        $display("%4t | %h | %h |  %b  | %h | 1.0 + 1.0 = 2.0 (Exp: 4800)", $time, a, b, sub, out);

        // Test Case 2: 1.0 - 1.0 = 0.0 (0x0000)
        a = 16'h4000; b = 16'h4000; sub = 1; #10;
        $display("%4t | %h | %h |  %b  | %h | 1.0 - 1.0 = 0.0 (Exp: 0000)", $time, a, b, sub, out);

        // Test Case 3: 4.0 + 1.0 = 5.0 (0x5200)
        // 4.0 in Posit(16,2) is 0x5000. 
        a = 16'h5000; b = 16'h4000; sub = 0; #10;
        $display("%4t | %h | %h |  %b  | %h | 4.0 + 1.0 = 5.0 (Exp: 5200)", $time, a, b, sub, out);

        // Test Case 4: 1.5 - 1.0 = 0.5 (0x3800)
        a = 16'h4400; b = 16'h4000; sub = 1; #10;
        $display("%4t | %h | %h |  %b  | %h | 1.5 - 1.0 = 0.5 (Exp: 3800)", $time, a, b, sub, out);

        // Test Case 5: 16.0 - 1.0 = 15.0 (0x5FC0)
        // 16.0 in Posit(16,2) is 0x6000.
        a = 16'h6000; b = 16'h4000; sub = 1; #10;
        $display("%4t | %h | %h |  %b  | %h | 16.0 - 1.0= 15.0 (Exp: 5f00)", $time, a, b, sub, out);

        $display("=========================================================");
        $finish;
    end

endmodule