`timescale 1ns / 1ps

module kalman_top_wrapper (
    input wire clk,
    input wire rst,
    input wire start,
    output wire done
);

    // Constant delta-t in Posit(16,2) format (0.1 = 16'h3800)
    wire [15:0] dt = 16'h3800;

    // Internal Wires from VIO
    wire [15:0] x_0_vio, x_1_vio;
    wire [15:0] z_0_vio, z_1_vio;
    wire [15:0] P_00_vio, P_01_vio, P_10_vio, P_11_vio;
    wire [15:0] Q_00_vio, Q_01_vio, Q_10_vio, Q_11_vio;
    wire [15:0] R_00_vio, R_01_vio, R_10_vio, R_11_vio;

    // Registered Datapath Signals (Breaks Timing Critical Path)
    reg [15:0] x_0, x_1;
    reg [15:0] z_0, z_1;
    reg [15:0] P_00, P_01, P_10, P_11;
    reg [15:0] Q_00, Q_01, Q_10, Q_11;
    reg [15:0] R_00, R_01, R_10, R_11;

    wire [15:0] x_out_0, x_out_1;
    wire [15:0] P_out_00, P_out_01, P_out_10, P_out_11;
    wire pipe_done;

    // VIO Soft Control Wires
    wire vio_start;
    wire vio_auto_mode; // Enable continuous execution

    // --- Double-Flop Synchronizer for VIO Controls ---
    reg vio_start_r1, vio_start_r2;
    reg vio_mode_r1,  vio_mode_r2;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vio_start_r1 <= 1'b0;
            vio_start_r2 <= 1'b0;
            vio_mode_r1  <= 1'b0;
            vio_mode_r2  <= 1'b0;
        end else begin
            vio_start_r1 <= vio_start;
            vio_start_r2 <= vio_start_r1;
            vio_mode_r1  <= vio_auto_mode;
            vio_mode_r2  <= vio_mode_r1;
        end
    end

    // --- Register Pipeline Layer for 16-bit VIO Outputs ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x_0  <= 16'd0; x_1  <= 16'd0;
            z_0  <= 16'd0; z_1  <= 16'd0;
            P_00 <= 16'd0; P_01 <= 16'd0; P_10 <= 16'd0; P_11 <= 16'd0;
            Q_00 <= 16'd0; Q_01 <= 16'd0; Q_10 <= 16'd0; Q_11 <= 16'd0;
            R_00 <= 16'd0; R_01 <= 16'd0; R_10 <= 16'd0; R_11 <= 16'd0;
        end else begin
            x_0  <= x_0_vio;  x_1  <= x_1_vio;
            z_0  <= z_0_vio;  z_1  <= z_1_vio;
            P_00 <= P_00_vio; P_01 <= P_01_vio; P_10 <= P_10_vio; P_11 <= P_11_vio;
            Q_00 <= Q_00_vio; Q_01 <= Q_01_vio; Q_10 <= Q_10_vio; Q_11 <= Q_11_vio;
            R_00 <= R_00_vio; R_01 <= R_01_vio; R_10 <= R_10_vio; R_11 <= R_11_vio;
        end
    end

    wire vio_start_edge = vio_start_r1 && !vio_start_r2;

    // --- Automated Trigger Generator ---
    reg auto_trigger;
    reg [1:0] wrapper_state;
    localparam W_IDLE = 2'b00, W_RUN = 2'b01, W_WAIT = 2'b10;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            auto_trigger  <= 1'b0;
            wrapper_state <= W_IDLE;
        end else begin
            auto_trigger <= 1'b0; // Default pulse high for 1 cycle
            
            case (wrapper_state)
                W_IDLE: begin
                    // Trigger automatically on hardware start, VIO edge, or Auto Mode
                    if (start || vio_start_edge || vio_mode_r2) begin
                        auto_trigger  <= 1'b1;
                        wrapper_state <= W_RUN;
                    end
                end
                
                W_RUN: begin
                    if (pipe_done) begin
                        wrapper_state <= W_WAIT;
                    end
                end
                
                W_WAIT: begin
                    // In continuous auto-mode, loop automatically
                    if (vio_mode_r2) begin
                        auto_trigger  <= 1'b1;
                        wrapper_state <= W_RUN;
                    end else if (!pipe_done) begin
                        wrapper_state <= W_IDLE;
                    end
                end
                
                default: wrapper_state <= W_IDLE;
            endcase
        end
    end

    assign done = pipe_done;

    // Instantiate Kalman Pipeline
    kalman_pipeline u_kalman (
        .clk(clk),
        .rst(rst),
        .start(auto_trigger),
        .x_prev_0(x_0),       .x_prev_1(x_1),
        .z_meas_0(z_0),       .z_meas_1(z_1),
        .P_prev_00(P_00),     .P_prev_01(P_01), 
        .P_prev_10(P_10),     .P_prev_11(P_11),
        .Q_00(Q_00),           .Q_01(Q_01), 
        .Q_10(Q_10),           .Q_11(Q_11),
        .R_00(R_00),           .R_01(R_01), 
        .R_10(R_10),           .R_11(R_11),
        .dt(dt),
        .x_out_0(x_out_0),     .x_out_1(x_out_1),
        .P_out_00(P_out_00),   .P_out_01(P_out_01),
        .P_out_10(P_out_10),   .P_out_11(P_out_11),
        .done(pipe_done)
    );

    // VIO IP Core Instance
    vio_0 u_vio (
        .clk(clk),
        
        .probe_in0(x_out_0),
        .probe_in1(x_out_1),
        .probe_in2(P_out_00),
        .probe_in3(P_out_01),
        .probe_in4(P_out_10),
        .probe_in5(P_out_11),
        .probe_in6(pipe_done),

        .probe_out0(x_0_vio),
        .probe_out1(x_1_vio),
        .probe_out2(z_0_vio),
        .probe_out3(z_1_vio),
        .probe_out4(P_00_vio),
        .probe_out5(P_01_vio),
        .probe_out6(P_10_vio),
        .probe_out7(P_11_vio),
        .probe_out8(Q_00_vio),
        .probe_out9(Q_01_vio),
        .probe_out10(Q_10_vio),
        .probe_out11(Q_11_vio),
        .probe_out12(R_00_vio),
        .probe_out13(R_01_vio),
        .probe_out14(R_10_vio),
        .probe_out15(R_11_vio),
        .probe_out16(vio_start),
        .probe_out17(vio_auto_mode) 
    );

endmodule