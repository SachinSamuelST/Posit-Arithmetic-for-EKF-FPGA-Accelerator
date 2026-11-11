# Posit-Arithmetic-for-EKF-FPGA-Accelerator
Pipelined hardware-software co-designed Sequential Extended Kalman Filter (SEKF) using Posit(16,2) arithmetic on a Xilinx Zynq-7000 SoC FPGA, featuring parameterizable Verilog ALUs and zero matrix inversion.

I. RTL Arithmetic Logic Unit (ALU) Design

1. posit_decoder.v: Unpacks 16-bit Posits into Sign, Regime ($k$), Exponent ($es=2$), and Mantissa fields using dynamic Leading Zero/One Detectors (LZD/LOD) and barrel shifters. Handles special state detection for Zero (0x0000) and NaR (0x8000).   
2. posit_encoder.v: Re-encodes internal 2's complement values back into tapered Posit format, managing scale-to-regime shifting and rounding.   
3. posit_add.v: 16-bit Posit Adder/Subtractor with dynamic scale difference alignment shifters ($\Delta\text{scale}$) and normalization logic.   
4. posit_mult.v: Pipelined multiplier featuring sign XOR logic, scale addition, $16 \times 16$ unsigned mantissa multiplication, and product scale adjustments.   
5. posit_div.v: Multi-cycle pipelined scalar divider executing scale subtraction, fixed-point integer quotient division ($f_A \ll N / f_B$), and divide-by-zero NaR protection.   
6. posit_alu.v: Central multiplexed wrapper routing opcodes (ADD, SUB, MUL, DIV) to respective arithmetic sub-cores


II. Sequential EKF Datapath & Top Wrapper

1. kalman_pipeline.v: An 88-state Finite State Machine (FSM) sequencing prediction ($\hat{x}^-, P^-$), Jacobian evaluation, and sequential scalar measurement updates across multi-cycle arithmetic pipelines.   
2. jacobian_eval.v: Hardcoded hardware evaluation engine generating system matrices $F_k$ and $H_k$ dynamically.   
3. matrix_mac_array.v: Parallel Posit matrix-vector MAC array for state updates.   
4. synchronizers.v: Double-flop synchronizers and rising-edge pulse detectors on asynchronous JTAG inputs (vio_start, vio_auto_mode) to eliminate clock-domain metastability.   
5. kalman_top_wrapper.v: Synthesizable top-level wrapper incorporating a 16-bit output register pipeline slice to sever critical routing paths and eliminate negative setup slack.


III. FPGA Prototyping & Vivado Results Summary

1. Target Compute Platform: Xilinx ZedBoard (Zynq-7000 SoC, Artix-7 PL fabric, xc7z020clg484-1).
2. Slice LUTs: 2,745 / 53,200 (5.16% utilization).
3. Slice Registers (FFs): 2,987 / 106,400 (2.81% utilization).
4. Total Slices: 1,174 / 13,300 (8.83% utilization).
5. DSP Slices: 1 / 220 (0.45% utilization).
6. Timing Closure: Closed at $T = 35\text{ ns}$ ($f_{\text{max}} = 28.57\text{ MHz}$) with positive slack: $\text{WNS} = +0.044\text{ ns}$, $\text{WHS} = +0.086\text{ ns}$, and zero failing endpoints.
7. Power Dissipation: $0.114\text{ W}$ Total On-Chip Power (Dynamic: $0.012\text{ W}$ / 10%, Static: $0.103\text{ W}$ / 90%).
8. Thermal Performance: Junction temperature at $26.3^\circ\text{C}$ ($1.3^\circ\text{C}$ above $25.0^\circ\text{C}$ ambient) with a $58.7^\circ\text{C}$ thermal margin.
9. In-System Debug: Verified on live silicon using Vivado Hardware Manager via Virtual I/O (VIO) probes (18 output probes, 7 input probes) and Integrated Logic Analyzer (ILA) traces.   
