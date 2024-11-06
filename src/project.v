/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module systolic_array_matrix_multiplier #(
    parameter N = 2,          // Dimension of the matrices
    parameter WIDTH = 2       // Bit-width of each matrix element
) (
    input  wire [7:0] ui_in,   // Dedicated input for matrix A and B (8 bits)
    output wire [7:0] uo_out,  // Dedicated output for matrix C (8 bits)
    input  wire [7:0] uio_in,   // General IO input (unused in this design)
    output wire [7:0] uio_out,  // General IO output (unused in this design)
    output wire [7:0] uio_oe,   // General IO output enable (unused in this design)
    input  wire       ena,      // Enable signal (always 1, can be ignored)
    input  wire       clk,      // Clock signal
    input  wire       rst_n     // Active low reset signal
);

    // Convert rst_n to active high reset
    wire rst = ~rst_n;

    // Internal signals
    reg [WIDTH-1:0] A [0:N-1][0:N-1]; // Registers for matrix A
    reg [WIDTH-1:0] B [0:N-1][0:N-1]; // Registers for matrix B
    reg [WIDTH*2-1:0] C [0:N-1][0:N-1]; // Registers for matrix C, allowing accumulation

    // Output registers
    reg [7:0] C_out; // Adjusted to 8-bit width

    // Assign outputs to top-level output ports
    assign uo_out = C_out; // Assign computed output matrix C to uo_out
    assign uio_out = 0;    // Unused output set to 0
    assign uio_oe = 0;     // Output enable set to 0

    // Data extraction from input
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ready_out <= 1;
            val_out <= 0;
        end else if (val_in && ready_out) begin
            // Extracting A and B elements from the 8-bit input
            A[0][0] <= ui_in[1:0];
            A[0][1] <= ui_in[3:2];
            A[1][0] <= ui_in[5:4];
            A[1][1] <= ui_in[7:6];

            B[0][0] <= ui_in[1:0];
            B[0][1] <= ui_in[3:2];
            B[1][0] <= ui_in[5:4];
            B[1][1] <= ui_in[7:6];

            ready_out <= 0; // Indicate data is being processed
        end else if (!ready_out && ready_in) begin
            ready_out <= 1; // Ready to accept new data after computation
            val_out <= 1; // Indicate that output data is ready
        end else if (val_out && ready_in) begin
            val_out <= 0; // Clear valid signal once output is acknowledged
        end
    end

    // Processing Element (PE) module
    module PE #(parameter WIDTH = 2) (
        input clk,
        input rst,
        input [WIDTH-1:0] a_in,
        input [WIDTH-1:0] b_in,
        input [WIDTH*2-1:0] c_in,
        output reg [WIDTH-1:0] a_out,
        output reg [WIDTH-1:0] b_out,
        output reg [WIDTH*2-1:0] c_out
    );
        always @(posedge clk or posedge rst) begin
            if (rst) begin
                c_out <= 0;
                a_out <= 0;
                b_out <= 0;
            end else begin
                c_out <= c_in + a_in * b_in;
                a_out <= a_in;
                b_out <= b_in;
            end
        end
    endmodule

    // Systolic Array Definition
    genvar i, j;
    generate
        for (i = 0; i < N; i = i + 1) begin : row
            for (j = 0; j < N; j = j + 1) begin : col
                wire [WIDTH-1:0] a_in, b_in, a_out, b_out;
                wire [WIDTH*2-1:0] c_in, c_out;

                // Input connections for systolic array
                if (j == 0) begin
                    assign a_in = A[i][j];
                end else begin
                    assign a_in = row[i].col[j-1].a_out;
                end

                if (i == 0) begin
                    assign b_in = B[i][j];
                end else begin
                    assign b_in = row[i-1].col[j].b_out;
                end

                assign c_in = (i == 0 || j == 0) ? 0 : row[i-1].col[j-1].c_out;

                PE #(.WIDTH(WIDTH)) pe (
                    .clk(clk),
                    .rst(rst),
                    .a_in(a_in),
                    .b_in(b_in),
                    .c_in(c_in),
                    .a_out(a_out),
                    .b_out(b_out),
                    .c_out(c_out)
                );

                always @(posedge clk or posedge rst) begin
                    if (rst) begin
                        C[i][j] <= 0;
                    end else begin
                        C[i][j] <= c_out;
                    end
                end
            end
        end
    endgenerate

    // Output assignment
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            C_out <= 0;
        end else if (!ready_out) begin
            C_out[3:0]   <= C[0][0];
            C_out[7:4]   <= C[0][1];
        end
    end

    // List all unused inputs to prevent warnings
    wire _unused = &{ena, clk, rst_n, 1'b0};

endmodule