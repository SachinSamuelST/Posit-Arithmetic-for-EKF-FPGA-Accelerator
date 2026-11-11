`timescale 1ns / 1ps

module posit_encoder #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire         sign,
    input  wire signed [8:0] scale,
    input  wire [2*N-1:0]    fraction,
    output wire [N-1:0]      posit_out
);

    // 1. Derive Regime Count (k) and Exponent (e)
    wire signed [6:0] k = scale >>> ES;
    wire [ES-1:0]     e = scale[ES-1:0];

    // 2. Generate Dynamic Regime Pattern
    wire [N-1:0] regime_bits = (k >= 0) ? ~({N{1'b1}} >> (k + 1)) : (1'b1 << (N - 1 - (-k)));
    wire [5:0]   regime_len  = (k >= 0) ? (k + 2) : (-k + 1);

    // 3. Align Exponent and Fraction Payload (FIXED)
    // Fills all 2*N bits so MSB aligns at bit 2*N-1 before shifting
    wire [2*N-1:0] payload     = {e, fraction[2*N-2 -: 2*N - ES]};
    wire [2*N-1:0] packed_body = ({regime_bits, {N{1'b0}}}) | (payload >> regime_len);

    // 4. Extract Top N-1 Body Bits (FIXED: Start at bit 2*N-1 to preserve bit 31)
    wire [N-2:0] abs_body = packed_body[2*N-1 -: N-1];

    // 5. Apply Two's Complement for Negative Values
    assign posit_out = sign ? (-{1'b0, abs_body}) : {1'b0, abs_body};

endmodule