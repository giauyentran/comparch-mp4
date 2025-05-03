`include "controller.sv"

module top(

    input logic clk
);


    logic [31:0] read_data;
    logic write_mem;
    logic [31:0] write_address;
    logic [31:0] write_data;
    logic [31:0] read_address;
    logic [2:0] funct3;
    logic [31:0] x;
    logic [31:0] y;
    logic [31:0] result;
    logic [31:0] result2;
    logic complete;
    logic complete2;
    logic rst;
    logic mulhsu;
    logic lui;

    mulhsu mulhsu1 (
        .clk (clk),
        .x (x),
        .y (y),
        .mulhsu (mulhsu),
        .result (result),
        .complete (complete),
        .rst (rst)
    );

    adder adder (
        .clk (clk),
        .read_data (read_data),
        .complete (complete),
        .complete2 (complete2),
        .read_address(read_address)
    );

    lui lui1 (
        .clk (clk),
        .x (x),
        .lui (lui),
        .result2 (result2),
        .complete2 (complete2),
        .rst (rst)
    );



    controller controller (
        .clk (clk),
        .read_data(read_data),
        .write_address(write_address),
        .write_data(write_data),
        .write_mem(write_mem),
        .funct3(funct3),
        .x (x),
        .y (y),
        .result (result),
        .result2 (result2),
        .complete (complete),
        .complete2 (complete2),
        .rst (rst),
        .mulhsu (mulhsu),
        .lui (lui)
    );

    memory #(
        .INIT_FILE      ("test")
    ) memory (
        .clk            (clk), 
        .read_address   (read_address), 
        .read_data      (read_data),
        .write_address(write_address),
        .write_data(write_data),
        .write_mem(write_mem),
        .funct3(funct3)
    );

endmodule