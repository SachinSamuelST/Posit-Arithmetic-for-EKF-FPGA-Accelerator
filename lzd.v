`timescale 1ns / 1ps

module lzd #(
    parameter WIDTH = 15
)(
    input  wire [WIDTH-1:0] in,
    output reg  [$clog2(WIDTH):0] count
);
    integer i;
    reg stop;

    always @(*) begin
        count = 0;
        stop = 0;
        for (i = WIDTH-1; i >= 0; i = i - 1) begin
            if (!stop) begin
                if (in[i] == 1'b0) begin
                    count = count + 1;
                end else begin
                    stop = 1;
                end
            end
        end
    end
endmodule