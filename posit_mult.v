`timescale 1ns / 1ps

module posit_mult #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire [N-1:0] a,
    input  wire [N-1:0] b,
    output wire [N-1:0] out
);

    // 1. Decode Input A and Input B
    wire sign_a, is_zero_a, is_nar_a;
    wire signed [5:0] regime_a;
    wire [ES-1:0] exp_a;
    wire [N-1:0] frac_a;

    wire sign_b, is_zero_b, is_nar_b;
    wire signed [5:0] regime_b;
    wire [ES-1:0] exp_b;
    wire [N-1:0] frac_b;

    posit_decoder #(.N(N), .ES(ES)) dec_a (
        .posit_in(a), .sign(sign_a), .is_zero(is_zero_a), 
        .is_nar(is_nar_a), .regime(regime_a), .exponent(exp_a), .fraction(frac_a)
    );

    posit_decoder #(.N(N), .ES(ES)) dec_b (
        .posit_in(b), .sign(sign_b), .is_zero(is_zero_b), 
        .is_nar(is_nar_b), .regime(regime_b), .exponent(exp_b), .fraction(frac_b)
    );

    // 2. Sign and Special Case Handling
    wire sign_out = sign_a ^ sign_b;
    wire is_zero_out = is_zero_a | is_zero_b;
    wire is_nar_out  = is_nar_a | is_nar_b;

    // 3. Scale Factor Addition
    wire signed [7:0] scale_a = (regime_a <<< ES) + $signed({1'b0, exp_a});
    wire signed [7:0] scale_b = (regime_b <<< ES) + $signed({1'b0, exp_b});
    wire signed [8:0] scale_sum = scale_a + scale_b;

    // 4. Fraction Multiplication
    wire [2*N-1:0] frac_mult = frac_a * frac_b;

    // 5. Normalization (FIXED: Corrected bit-slice starting indices)
    wire mult_overflow = frac_mult[2*N-1];
    wire signed [8:0] final_scale = mult_overflow ? (scale_sum + 1) : scale_sum;
    wire [N-1:0] norm_frac = mult_overflow ? frac_mult[2*N-1 -: N] : frac_mult[2*N-2 -: N];

    // 6. Encode Output
    wire [N-1:0] encoded_out;

    posit_encoder #(.N(N), .ES(ES)) enc (
        .sign(sign_out),
        .scale(final_scale),
        .fraction({norm_frac, {N{1'b0}}}),
        .posit_out(encoded_out)
    );

    assign out = is_zero_out ? {N{1'b0}} : 
                 is_nar_out  ? {1'b1, {(N-1){1'b0}}} : encoded_out;

endmodule