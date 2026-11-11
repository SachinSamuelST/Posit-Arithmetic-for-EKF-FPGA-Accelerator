`timescale 1ns / 1ps

module tb_kalman_pipeline();

    // Parameters
    parameter N = 16;
    parameter ES = 2;

    // Clock and Control
    reg clk;
    reg rst;
    reg start;

    // Posit Constants for Simulation
    localparam POSIT_ZERO = 16'h0000; // 0.0
    localparam POSIT_ONE  = 16'h4000; // 1.0
    localparam POSIT_HALF = 16'h3000; // 0.5
    localparam POSIT_DT   = 16'h3800; // 0.1 (example)

    // DUT Inputs
    reg [N-1:0] x_prev_0, x_prev_1;
    reg [N-1:0] z_meas_0, z_meas_1;
    reg [N-1:0] P_prev_00, P_prev_01, P_prev_10, P_prev_11;
    reg [N-1:0] Q_00, Q_01, Q_10, Q_11;
    reg [N-1:0] R_00, R_01, R_10, R_11;
    reg [N-1:0] dt;

    // DUT Outputs
    wire [N-1:0] x_out_0, x_out_1;
    wire [N-1:0] P_out_00, P_out_01, P_out_10, P_out_11;
    wire done;

    // Instantiate the Pipeline
    kalman_pipeline #(
        .N(N),
        .ES(ES)
    ) uut (
        .clk(clk),
        .rst(rst),
        .start(start),
        
        .x_prev_0(x_prev_0), .x_prev_1(x_prev_1),
        .z_meas_0(z_meas_0), .z_meas_1(z_meas_1),
        
        .P_prev_00(P_prev_00), .P_prev_01(P_prev_01), 
        .P_prev_10(P_prev_10), .P_prev_11(P_prev_11),
        
        .Q_00(Q_00), .Q_01(Q_01), .Q_10(Q_10), .Q_11(Q_11),
        .R_00(R_00), .R_01(R_01), .R_10(R_10), .R_11(R_11),
        .dt(dt),
        
        .x_out_0(x_out_0), .x_out_1(x_out_1),
        .P_out_00(P_out_00), .P_out_01(P_out_01), 
        .P_out_10(P_out_10), .P_out_11(P_out_11),
        .done(done)
    );

    // Clock Generation (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test Sequence
    initial begin
        // Initialize Inputs
        rst = 1;
        start = 0;
        
        // Identity Matrix for P_prev
        P_prev_00 = POSIT_ONE;  P_prev_01 = POSIT_ZERO;
        P_prev_10 = POSIT_ZERO; P_prev_11 = POSIT_ONE;
        
        // Small Noise for Q
        Q_00 = POSIT_HALF; Q_01 = POSIT_ZERO;
        Q_10 = POSIT_ZERO; Q_11 = POSIT_HALF;
        
        // Small Noise for R
        R_00 = POSIT_HALF; R_01 = POSIT_ZERO;
        R_10 = POSIT_ZERO; R_11 = POSIT_HALF;
        
        // Initial State and Measurements
        x_prev_0 = POSIT_ONE; x_prev_1 = POSIT_ZERO;
        z_meas_0 = POSIT_ONE; z_meas_1 = POSIT_ZERO;
        dt = POSIT_DT;

        // Apply Reset
        #20;
        rst = 0;
        #20;

        // Trigger the FSM
        start = 1;
        #10;
        start = 0; // De-assert start after 1 clock cycle

        // Wait for the pipeline to complete
        wait(done == 1'b1);
        
        #50;
        $display("Simulation Complete: EKF iteration finished.");
        $stop;
    end

endmodule