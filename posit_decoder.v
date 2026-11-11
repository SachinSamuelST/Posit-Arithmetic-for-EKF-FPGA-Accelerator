`timescale 1ns / 1ps

module posit_decoder #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire [N-1:0] posit_in,
    output wire         sign,
    output wire         is_zero,
    output wire         is_nar,
    output wire signed [5:0] regime,
    output wire [ES-1:0]      exponent,
    output wire [N-1:0]      fraction
);

    // 1. Identify Special Cases
    assign is_zero = ~(|posit_in);
    assign is_nar  = posit_in[N-1] & ~(|(posit_in[N-2:0]));
    assign sign    = posit_in[N-1];

    // 2. Compute 2's Complement Absolute Value for Negative Posits
    wire [N-1:0] abs_in = sign ? (-posit_in) : posit_in;
    wire [N-2:0] body   = abs_in[N-2:0]; // Exclude sign bit

    // 3. Count Regime Run-Length using LZD
    wire r_bit = body[N-2];
    
    // Normalize body input so LZD always counts leading zeros of the run
    wire [N-2:0] lzd_in = r_bit ? ~body : body;

    wire [$clog2(N-1):0] r_count;

    lzd #(.WIDTH(N-1)) lzd_inst (
        .in(lzd_in),
        .count(r_count)
    );

    // 4. Calculate Signed Regime Value (k)
    // If r_bit == 1: k = r_count - 1
    // If r_bit == 0: k = -r_count
    assign regime = r_bit ? $signed({1'b0, r_count}) - 1 : -$signed({1'b0, r_count});

    // 5. Dynamic Bit Shifting to Extract Exponent & Fraction
    wire [2*N-1:0] body_ext     = {body, {N{1'b0}}};
    
    // Shift left past the regime bits AND the 1-bit terminator (r_count + 1)
    wire [2*N-1:0] shifted_body = body_ext << (r_count + 1);

    // Extract Exponent bits
    assign exponent = shifted_body[2*N-2 -: ES];
    
    // Concatenate hidden leading bit 1 with remaining fraction payload
    assign fraction = {1'b1, shifted_body[2*N-2-ES -: N-1]};

endmodule