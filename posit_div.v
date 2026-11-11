`timescale 1ns / 1ps

module posit_div #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    input  wire [N-1:0] a,
    input  wire [N-1:0] b,
    output reg  [N-1:0] out,
    output reg          done
);

    // FSM States
    localparam IDLE   = 2'b00;
    localparam CALC   = 2'b01;
    localparam FINISH = 2'b10;

    reg [1:0] state;
    reg [5:0] count;

    // Internal registers for multi-cycle execution
    reg sign_out;
    reg is_nar_out;
    reg is_zero_out;
    reg signed [8:0] scale_sub;
    reg [N-1:0] divisor;
    reg [3*N-1:0] rem_quot;

    // 1. Decode Input A and Input B (Combinational)
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

    // Scale Factor Calculation
    wire signed [7:0] scale_a = (regime_a <<< ES) + $signed({1'b0, exp_a});
    wire signed [7:0] scale_b = (regime_b <<< ES) + $signed({1'b0, exp_b});

    // --- Declarations moved up to fix [Synth 8-6901] ---
    wire [2*N-1:0] div_quotient = rem_quot[2*N-1:0];
    wire div_overflow = div_quotient[N];
    wire signed [8:0] final_scale = div_overflow ? scale_sub : (scale_sub - 1);
    wire [N-1:0] norm_frac = div_overflow ? div_quotient[N -: N] : div_quotient[N-1 -: N];

    wire [N-1:0] encoded_out;

    posit_encoder #(.N(N), .ES(ES)) enc (
        .sign(sign_out),
        .scale(final_scale),
        .fraction({norm_frac, {N{1'b0}}}),
        .posit_out(encoded_out)
    );

    // Fixed: Converted temp_shift to combinational wire outside sequential block
    wire [3*N-1:0] temp_shift = rem_quot << 1;

    // 2. Multi-Cycle Iterative Restoring Division FSM
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= IDLE;
            count       <= 6'd0;
            done        <= 1'b0;
            out         <= {N{1'b0}};
            rem_quot    <= {3*N{1'b0}};
            divisor     <= {N{1'b0}};
            sign_out    <= 1'b0;
            is_nar_out  <= 1'b0;
            is_zero_out <= 1'b0;
            scale_sub   <= 9'd0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        sign_out    <= sign_a ^ sign_b;
                        is_nar_out  <= is_nar_a | is_nar_b | is_zero_b;
                        is_zero_out <= is_zero_a & ~(is_nar_a | is_nar_b | is_zero_b);
                        scale_sub   <= scale_a - scale_b;
                        divisor     <= frac_b;
                        rem_quot    <= {{N{1'b0}}, frac_a, {N{1'b0}}};
                        count       <= 6'd0;
                        state       <= CALC;
                    end
                end

                CALC: begin
                    if (temp_shift[3*N-1 : 2*N] >= divisor) begin
                        rem_quot <= {temp_shift[3*N-1 : 2*N] - divisor, temp_shift[2*N-1 : 1], 1'b1};
                    end else begin
                        rem_quot <= temp_shift;
                    end

                    if (count == (2*N - 1)) begin
                        state <= FINISH;
                    end else begin
                        count <= count + 1'b1;
                    end
                end

                FINISH: begin
                    if (is_nar_out) begin
                        out <= {1'b1, {(N-1){1'b0}}};
                    end else if (is_zero_out) begin
                        out <= {N{1'b0}};
                    end else begin
                        out <= encoded_out;
                    end
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule