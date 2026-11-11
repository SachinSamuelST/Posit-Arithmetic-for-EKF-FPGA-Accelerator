`timescale 1ns / 1ps

module posit_add #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire [N-1:0] a,
    input  wire [N-1:0] b,
    input  wire         sub,
    output wire [N-1:0] out
);

    // 1. Decode Inputs
    wire sign_a, is_zero_a, is_nar_a;
    wire signed [5:0] regime_a;
    wire [ES-1:0] exp_a;
    wire [N-1:0] frac_a;

    wire sign_b_raw, is_zero_b, is_nar_b;
    wire signed [5:0] regime_b;
    wire [ES-1:0] exp_b;
    wire [N-1:0] frac_b;

    posit_decoder #(.N(N), .ES(ES)) dec_a (
        .posit_in(a), .sign(sign_a), .is_zero(is_zero_a), 
        .is_nar(is_nar_a), .regime(regime_a), .exponent(exp_a), .fraction(frac_a)
    );

    posit_decoder #(.N(N), .ES(ES)) dec_b (
        .posit_in(b), .sign(sign_b_raw), .is_zero(is_zero_b), 
        .is_nar(is_nar_b), .regime(regime_b), .exponent(exp_b), .fraction(frac_b)
    );

    wire sign_b = sign_b_raw ^ sub;

    // 2. Scale Factors & Alignment
    wire signed [7:0] scale_a = (regime_a <<< ES) + $signed({1'b0, exp_a});
    wire signed [7:0] scale_b = (regime_b <<< ES) + $signed({1'b0, exp_b});

    wire a_gt_b = (scale_a > scale_b) || ((scale_a == scale_b) && (frac_a >= frac_b));
    wire [7:0] scale_diff = a_gt_b ? (scale_a - scale_b) : (scale_b - scale_a);

    wire [2*N-1:0] frac_a_ext = {frac_a, {N{1'b0}}};
    wire [2*N-1:0] frac_b_ext = {frac_b, {N{1'b0}}};

    wire [2*N-1:0] aligned_a = a_gt_b ? frac_a_ext : (frac_a_ext >> scale_diff);
    wire [2*N-1:0] aligned_b = a_gt_b ? (frac_b_ext >> scale_diff) : frac_b_ext;

    // 3. Fraction Computation
    wire res_sign = a_gt_b ? sign_a : sign_b;
    wire signed [8:0] base_scale = a_gt_b ? scale_a : scale_b;
    wire [2*N:0] frac_sum = (sign_a == sign_b) ? (aligned_a + aligned_b) : 
                            (a_gt_b ? (aligned_a - aligned_b) : (aligned_b - aligned_a));

    // 4. Normalization & Carry Handling (FIXED: Isolated LZD bug)
    wire [$clog2(2*N):0] lz_count;
    
    // Instantiate LZD to count identical MSBs
    lzd #(.WIDTH(2*N)) norm_lzd (
        .in(frac_sum[2*N-1:0]),
        .count(lz_count)
    );

    wire sum_overflow = frac_sum[2*N];
    
    // If MSB is 1, it's already normalized (shift = 0). Otherwise, use lz_count.
    wire [$clog2(2*N):0] shift_amount = sum_overflow ? 0 : 
                                        (frac_sum[2*N-1] ? 0 : lz_count);

    wire signed [8:0] norm_scale = sum_overflow ? (base_scale + 1) : 
                                   (frac_sum[2*N-1:0] == 0) ? 0 : (base_scale - shift_amount);
                                   
    wire [2*N-1:0] norm_frac = sum_overflow ? frac_sum[2*N : 1] : 
                               (frac_sum[2*N-1:0] << shift_amount);

    // 5. Encode Output
    wire [N-1:0] encoded_out;
    posit_encoder #(.N(N), .ES(ES)) enc (
        .sign(res_sign),
        .scale(norm_scale),
        .fraction(norm_frac),
        .posit_out(encoded_out)
    );

    wire is_zero_out = (is_zero_a && is_zero_b) || (frac_sum == 0);
    wire is_nar_out  = is_nar_a || is_nar_b;

    assign out = is_zero_out ? {N{1'b0}} :
                 is_nar_out  ? {1'b1, {(N-1){1'b0}}} : encoded_out;

endmodule