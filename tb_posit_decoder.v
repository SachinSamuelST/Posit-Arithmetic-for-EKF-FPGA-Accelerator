`timescale 1ns / 1ps

module tb_posit_decoder();

    // 1. Declare Testbench Signals
    reg  [15:0] posit_in;
    wire        sign;
    wire        is_zero;
    wire        is_nar;
    wire signed [5:0] regime;
    wire [1:0]  exponent;
    wire [15:0] fraction;

    // 2. Instantiate the Unit Under Test (UUT)
    posit_decoder #(
        .N(16),
        .ES(2)
    ) uut (
        .posit_in(posit_in),
        .sign(sign),
        .is_zero(is_zero),
        .is_nar(is_nar),
        .regime(regime),
        .exponent(exponent),
        .fraction(fraction)
    );

    // 3. Apply Test Vectors
    initial begin
        // Print a header for the console output
        $display("==========================================================================");
        $display("Time | Posit In | Sign | Zero | NaR | Regime (k) | Exponent | Fraction");
        $display("==========================================================================");
        
        // Monitor changes and print them automatically
        $monitor("%4t |   %04X   |   %b  |   %b  |  %b  |     %d     |    %b    | %b", 
                 $time, posit_in, sign, is_zero, is_nar, regime, exponent, fraction);

        // Test Case 1: Zero (0x0000)
        posit_in = 16'h0000;
        #10;

        // Test Case 2: Not-a-Real / Infinity (0x8000)
        posit_in = 16'h8000;
        #10;

        // Test Case 3: +1.0 (0x4000)
        // Should yield: Sign=0, Regime=0, Exponent=0
        posit_in = 16'h4000;
        #10;

        // Test Case 4: -1.0 (0xC000)
        // Should yield: Sign=1, Regime=0, Exponent=0
        posit_in = 16'hC000;
        #10;

        // Test Case 5: +4.0 (0x6000)
        // Should yield: Sign=0, Regime=1, Exponent=0
        posit_in = 16'h6000;
        #10;

        $display("==========================================================================");
        $display("Simulation Complete.");
        $finish;
    end

endmodule