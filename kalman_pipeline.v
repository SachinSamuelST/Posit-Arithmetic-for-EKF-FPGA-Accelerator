`timescale 1ns / 1ps

module kalman_pipeline #(
    parameter N = 16,
    parameter ES = 2
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          start,
    
    // Inputs (2x1 vectors and 2x2 matrices)
    input  wire [N-1:0] x_prev_0, x_prev_1,
    input  wire [N-1:0] z_meas_0, z_meas_1,
    input  wire [N-1:0] P_prev_00, P_prev_01, P_prev_10, P_prev_11,
    input  wire [N-1:0] Q_00, Q_01, Q_10, Q_11,
    input  wire [N-1:0] R_00, R_01, R_10, R_11,
    input  wire [N-1:0] dt,

    // Outputs
    output reg  [N-1:0] x_out_0, x_out_1,
    output reg  [N-1:0] P_out_00, P_out_01, P_out_10, P_out_11,
    output reg          done
);

    // Posit 16-bit constants
    localparam POSIT_ONE  = 16'h4000;
    localparam POSIT_ZERO = 16'h0000;

    // ALU Operations
    localparam OP_ADD = 2'b00;
    localparam OP_SUB = 2'b01;
    localparam OP_MUL = 2'b10;
    localparam OP_DIV = 2'b11;

    // FSM State Encoding (Updated to 7-bit literals)
    reg [6:0] state;

    localparam IDLE           = 7'd0;
    
    // 1. State Prediction
    localparam PX_MUL_A       = 7'd1;
    localparam PX_MUL_B       = 7'd2;
    localparam PX_ADD_B       = 7'd3;

    // 2. Covariance Prediction
    localparam PP_M1_00_A     = 7'd4;
    localparam PP_M1_00_B     = 7'd5;
    localparam PP_M1_00_ADD   = 7'd6;
    localparam PP_M1_01_A     = 7'd7;
    localparam PP_M1_01_B     = 7'd8;
    localparam PP_M1_01_ADD   = 7'd9;

    localparam PP_M2_00_A     = 7'd10;
    localparam PP_M2_00_B     = 7'd11;
    localparam PP_M2_00_ADD   = 7'd12;
    localparam PP_M2_00_Q     = 7'd13;

    localparam PP_M2_01_A     = 7'd14;
    localparam PP_M2_01_Q     = 7'd15;

    localparam PP_M2_10_A     = 7'd16;
    localparam PP_M2_10_B     = 7'd17;
    localparam PP_M2_10_ADD   = 7'd18;
    localparam PP_M2_10_Q     = 7'd19;

    localparam PP_M2_11_A     = 7'd20;
    localparam PP_M2_11_Q     = 7'd21;

    // 3. Residual
    localparam RESID_0_A      = 7'd22;
    localparam RESID_0_B      = 7'd23;
    localparam RESID_1_B      = 7'd24;

    // 4. Innovation Covariance
    localparam S_MAT_00_A     = 7'd25;
    localparam S_MAT_00_B     = 7'd26;
    localparam S_MAT_01_B     = 7'd27;
    localparam S_MAT_10_B     = 7'd28;
    localparam S_MAT_11_B     = 7'd29;

    // 5. Determinant & Inversion
    localparam DET_MUL1_A     = 7'd30;
    localparam DET_MUL1_B     = 7'd31;
    localparam DET_MUL2_B     = 7'd32;
    localparam DET_SUB_B      = 7'd33;
    localparam DET_DIV_TRIG   = 7'd70;
    localparam DET_INV_B      = 7'd34;

    localparam SINV_00_A      = 7'd35;
    localparam SINV_00_B      = 7'd36;
    localparam SINV_01_NEG    = 7'd37;
    localparam SINV_01_B      = 7'd38;
    localparam SINV_10_NEG    = 7'd39;
    localparam SINV_10_B      = 7'd40;
    localparam SINV_11_B      = 7'd41;

    // 6. Kalman Gain
    localparam K_00_M1_A      = 7'd42;
    localparam K_00_M1_B      = 7'd43;
    localparam K_00_M2_B      = 7'd44;
    localparam K_00_ADD_B     = 7'd45;

    localparam K_01_M1_A      = 7'd46;
    localparam K_01_M1_B      = 7'd47;
    localparam K_01_M2_B      = 7'd48;
    localparam K_01_ADD_B     = 7'd49;

    localparam K_10_M1_A      = 7'd50;
    localparam K_10_M1_B      = 7'd51;
    localparam K_10_M2_B      = 7'd52;
    localparam K_10_ADD_B     = 7'd53;

    localparam K_11_M1_A      = 7'd54;
    localparam K_11_M1_B      = 7'd55;
    localparam K_11_M2_B      = 7'd56;
    localparam K_11_ADD_B     = 7'd57;

    // 7. State Update
    localparam XUPD_0_M1_A    = 7'd58;
    localparam XUPD_0_M1_B    = 7'd59;
    localparam XUPD_0_M2_B    = 7'd60;
    localparam XUPD_0_ADD1_B  = 7'd61;
    localparam XUPD_0_ADD2_B  = 7'd62;

    localparam XUPD_1_M1_A    = 7'd63;
    localparam XUPD_1_M1_B    = 7'd64;
    localparam XUPD_1_M2_B    = 7'd65;
    localparam XUPD_1_ADD1_B  = 7'd66;
    localparam XUPD_1_ADD2_B  = 7'd67;

    // 8. Output Update & Done State
    localparam PUPD_STATE     = 7'd68;
    localparam DONE_STATE     = 7'd69;

    // Internal registers
    reg [N-1:0] x_pred_0, x_pred_1;
    reg [N-1:0] P_pred_00, P_pred_01, P_pred_10, P_pred_11;
    reg [N-1:0] T1_00, T1_01;
    reg [N-1:0] y_0, y_1;
    reg [N-1:0] S_00, S_01, S_10, S_11;
    reg [N-1:0] S_inv_00, S_inv_01, S_inv_10, S_inv_11;
    reg [N-1:0] K_00, K_01, K_10, K_11;
    reg [N-1:0] temp1, temp2, det_S, inv_det_S;
    reg [N-1:0] neg_S_01, neg_S_10;

    // Posit ALU handshaking signals
    reg          alu_start;
    wire         alu_done;
    reg  [N-1:0] alu_a, alu_b;
    reg  [1:0]   alu_op;
    wire [N-1:0] alu_out;

    posit_alu #(.N(N), .ES(ES)) u_alu (
        .clk(clk),
        .rst(rst),
        .start(alu_start),
        .a(alu_a), 
        .b(alu_b), 
        .op(alu_op), 
        .out(alu_out),
        .done(alu_done)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            done <= 1'b0;
            alu_start <= 1'b0;
            alu_a <= 0; alu_b <= 0; alu_op <= OP_ADD;
            x_out_0 <= 0; x_out_1 <= 0;
            P_out_00 <= 0; P_out_01 <= 0; P_out_10 <= 0; P_out_11 <= 0;
            x_pred_0 <= 0; x_pred_1 <= 0;
            P_pred_00 <= 0; P_pred_01 <= 0; P_pred_10 <= 0; P_pred_11 <= 0;
            T1_00 <= 0; T1_01 <= 0;
            y_0 <= 0; y_1 <= 0;
            S_00 <= 0; S_01 <= 0; S_10 <= 0; S_11 <= 0;
            S_inv_00 <= 0; S_inv_01 <= 0; S_inv_10 <= 0; S_inv_11 <= 0;
            K_00 <= 0; K_01 <= 0; K_10 <= 0; K_11 <= 0;
            temp1 <= 0; temp2 <= 0; det_S <= 0; inv_det_S <= 0;
            neg_S_01 <= 0; neg_S_10 <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    alu_start <= 1'b0;
                    if (start) begin
                        alu_a <= dt; alu_b <= x_prev_1; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= PX_MUL_A;
                    end
                end

                // --- 1. State Prediction ---
                PX_MUL_A: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp1 <= alu_out;
                        alu_a <= x_prev_0; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= PX_MUL_B;
                    end
                end
                PX_MUL_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        x_pred_0 <= alu_out;
                        x_pred_1 <= x_prev_1;
                        alu_a <= dt; alu_b <= P_prev_10; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= PX_ADD_B;
                    end
                end
                PX_ADD_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= PP_M1_00_A;
                    end
                end

                // --- 2. Covariance Prediction ---
                PP_M1_00_A: begin
                    alu_a <= P_prev_00; alu_b <= alu_out; alu_op <= OP_ADD;
                    alu_start <= 1'b1;
                    state <= PP_M1_00_B;
                end
                PP_M1_00_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        T1_00 <= alu_out;
                        alu_a <= dt; alu_b <= P_prev_11; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= PP_M1_00_ADD;
                    end
                end
                PP_M1_00_ADD: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= PP_M1_01_A;
                    end
                end
                PP_M1_01_A: begin
                    alu_a <= P_prev_01; alu_b <= alu_out; alu_op <= OP_ADD;
                    alu_start <= 1'b1;
                    state <= PP_M1_01_B;
                end
                PP_M1_01_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        T1_01 <= alu_out;
                        alu_a <= dt; alu_b <= alu_out; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= PP_M1_01_ADD;
                    end
                end
                PP_M1_01_ADD: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= PP_M2_00_A;
                    end
                end

                PP_M2_00_A: begin
                    alu_a <= T1_00; alu_b <= alu_out; alu_op <= OP_ADD;
                    alu_start <= 1'b1;
                    state <= PP_M2_00_B;
                end
                PP_M2_00_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        alu_a <= alu_out; alu_b <= Q_00; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= PP_M2_00_ADD;
                    end
                end
                PP_M2_00_ADD: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        P_pred_00 <= alu_out;
                        alu_a <= T1_01; alu_b <= Q_01; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= PP_M2_00_Q;
                    end
                end
                PP_M2_00_Q: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        P_pred_01 <= alu_out;
                        alu_a <= dt; alu_b <= P_prev_11; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= PP_M2_01_A;
                    end
                end

                PP_M2_01_A: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= PP_M2_01_Q;
                    end
                end
                PP_M2_01_Q: begin
                    alu_a <= P_prev_10; alu_b <= alu_out; alu_op <= OP_ADD;
                    alu_start <= 1'b1;
                    state <= PP_M2_10_A;
                end

                PP_M2_10_A: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        alu_a <= alu_out; alu_b <= Q_10; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= PP_M2_10_B;
                    end
                end
                PP_M2_10_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        P_pred_10 <= alu_out;
                        alu_a <= P_prev_11; alu_b <= Q_11; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= PP_M2_10_ADD;
                    end
                end
                PP_M2_10_ADD: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= PP_M2_10_Q;
                    end
                end
                PP_M2_10_Q: begin
                    P_pred_11 <= alu_out;
                    alu_a <= z_meas_0; alu_b <= x_pred_0; alu_op <= OP_SUB;
                    alu_start <= 1'b1;
                    state <= PP_M2_11_A;
                end

                PP_M2_11_A: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= PP_M2_11_Q;
                    end
                end
                PP_M2_11_Q: begin
                    state <= RESID_0_A;
                end

                // --- 3. Residual Measurement ---
                RESID_0_A: begin
                    y_0 <= alu_out;
                    alu_a <= z_meas_1; alu_b <= x_pred_1; alu_op <= OP_SUB;
                    alu_start <= 1'b1;
                    state <= RESID_0_B;
                end
                RESID_0_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        y_1 <= alu_out;
                        alu_a <= P_pred_00; alu_b <= R_00; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= RESID_1_B;
                    end
                end
                RESID_1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= S_MAT_00_A;
                    end
                end

                // --- 4. Innovation Covariance S ---
                S_MAT_00_A: begin
                    S_00 <= alu_out;
                    alu_a <= P_pred_01; alu_b <= R_01; alu_op <= OP_ADD;
                    alu_start <= 1'b1;
                    state <= S_MAT_00_B;
                end
                S_MAT_00_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_01 <= alu_out;
                        alu_a <= P_pred_10; alu_b <= R_10; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= S_MAT_01_B;
                    end
                end
                S_MAT_01_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_10 <= alu_out;
                        alu_a <= P_pred_11; alu_b <= R_11; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= S_MAT_10_B;
                    end
                end
                S_MAT_10_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_11 <= alu_out;
                        alu_a <= alu_out; alu_b <= S_00; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= S_MAT_11_B;
                    end
                end
                S_MAT_11_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= DET_MUL1_A;
                    end
                end

                // --- 5. Determinant & Matrix Inverse ---
                DET_MUL1_A: begin
                    temp1 <= alu_out;
                    alu_a <= S_01; alu_b <= S_10; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= DET_MUL1_B;
                end
                DET_MUL1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_SUB;
                        alu_start <= 1'b1;
                        state <= DET_MUL2_B;
                    end
                end
                DET_MUL2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        det_S <= alu_out;
                        alu_a <= POSIT_ONE; alu_b <= alu_out; alu_op <= OP_DIV;
                        alu_start <= 1'b1;
                        state <= DET_SUB_B;
                    end
                end
                DET_SUB_B: begin
                    alu_start <= 1'b0;
                    state <= DET_DIV_TRIG;
                end
                DET_DIV_TRIG: begin
                    state <= DET_INV_B;
                end
                DET_INV_B: begin
                    if (alu_done) begin
                        inv_det_S <= alu_out;
                        alu_a <= S_11; alu_b <= alu_out; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= SINV_00_A;
                    end
                end

                SINV_00_A: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_inv_00 <= alu_out;
                        alu_a <= POSIT_ZERO; alu_b <= S_01; alu_op <= OP_SUB;
                        alu_start <= 1'b1;
                        state <= SINV_00_B;
                    end
                end
                SINV_00_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        neg_S_01 <= alu_out;
                        alu_a <= alu_out; alu_b <= inv_det_S; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= SINV_01_NEG;
                    end
                end
                SINV_01_NEG: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_inv_01 <= alu_out;
                        alu_a <= POSIT_ZERO; alu_b <= S_10; alu_op <= OP_SUB;
                        alu_start <= 1'b1;
                        state <= SINV_01_B;
                    end
                end
                SINV_01_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        neg_S_10 <= alu_out;
                        alu_a <= alu_out; alu_b <= inv_det_S; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= SINV_10_NEG;
                    end
                end
                SINV_10_NEG: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_inv_10 <= alu_out;
                        alu_a <= S_00; alu_b <= inv_det_S; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= SINV_10_B;
                    end
                end
                SINV_10_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        S_inv_11 <= alu_out;
                        alu_a <= P_pred_00; alu_b <= S_inv_00; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= SINV_11_B;
                    end
                end
                SINV_11_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= K_00_M1_A;
                    end
                end

                // --- 6. Kalman Gain ---
                K_00_M1_A: begin
                    temp1 <= alu_out;
                    alu_a <= P_pred_01; alu_b <= S_inv_10; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= K_00_M1_B;
                end
                K_00_M1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= K_00_M2_B;
                    end
                end
                K_00_M2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        K_00 <= alu_out;
                        alu_a <= P_pred_00; alu_b <= S_inv_01; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= K_00_ADD_B;
                    end
                end
                K_00_ADD_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= K_01_M1_A;
                    end
                end

                K_01_M1_A: begin
                    temp1 <= alu_out;
                    alu_a <= P_pred_01; alu_b <= S_inv_11; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= K_01_M1_B;
                end
                K_01_M1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= K_01_M2_B;
                    end
                end
                K_01_M2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        K_01 <= alu_out;
                        alu_a <= P_pred_10; alu_b <= S_inv_00; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= K_01_ADD_B;
                    end
                end
                K_01_ADD_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= K_10_M1_A;
                    end
                end

                K_10_M1_A: begin
                    temp1 <= alu_out;
                    alu_a <= P_pred_11; alu_b <= S_inv_10; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= K_10_M1_B;
                end
                K_10_M1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= K_10_M2_B;
                    end
                end
                K_10_M2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        K_10 <= alu_out;
                        alu_a <= P_pred_10; alu_b <= S_inv_01; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= K_10_ADD_B;
                    end
                end
                K_10_ADD_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= K_11_M1_A;
                    end
                end

                K_11_M1_A: begin
                    temp1 <= alu_out;
                    alu_a <= P_pred_11; alu_b <= S_inv_11; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= K_11_M1_B;
                end
                K_11_M1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= K_11_M2_B;
                    end
                end
                K_11_M2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        K_11 <= alu_out;
                        alu_a <= K_00; alu_b <= y_0; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= K_11_ADD_B;
                    end
                end
                K_11_ADD_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= XUPD_0_M1_A;
                    end
                end

                // --- 7. State Vector Update ---
                XUPD_0_M1_A: begin
                    temp1 <= alu_out;
                    alu_a <= K_01; alu_b <= y_1; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= XUPD_0_M1_B;
                end
                XUPD_0_M1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= XUPD_0_M2_B;
                    end
                end
                XUPD_0_M2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        alu_a <= x_pred_0; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= XUPD_0_ADD1_B;
                    end
                end
                XUPD_0_ADD1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        x_out_0 <= alu_out;
                        alu_a <= K_10; alu_b <= y_0; alu_op <= OP_MUL;
                        alu_start <= 1'b1;
                        state <= XUPD_0_ADD2_B;
                    end
                end
                XUPD_0_ADD2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        state <= XUPD_1_M1_A;
                    end
                end

                XUPD_1_M1_A: begin
                    temp1 <= alu_out;
                    alu_a <= K_11; alu_b <= y_1; alu_op <= OP_MUL;
                    alu_start <= 1'b1;
                    state <= XUPD_1_M1_B;
                end
                XUPD_1_M1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        temp2 <= alu_out;
                        alu_a <= temp1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= XUPD_1_M2_B;
                    end
                end
                XUPD_1_M2_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        alu_a <= x_pred_1; alu_b <= alu_out; alu_op <= OP_ADD;
                        alu_start <= 1'b1;
                        state <= XUPD_1_ADD1_B;
                    end
                end
                XUPD_1_ADD1_B: begin
                    alu_start <= 1'b0;
                    if (alu_done) begin
                        x_out_1 <= alu_out;
                        state <= XUPD_1_ADD2_B;
                    end
                end
                XUPD_1_ADD2_B: begin
                    state <= PUPD_STATE;
                end

                // --- 8. Covariance Output Update ---
                PUPD_STATE: begin
                    P_out_00 <= P_pred_00;
                    P_out_01 <= P_pred_01;
                    P_out_10 <= P_pred_10;
                    P_out_11 <= P_pred_11;
                    done     <= 1'b1;
                    state    <= DONE_STATE;
                end

                DONE_STATE: begin
                    done <= 1'b1;
                    if (!start) state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule