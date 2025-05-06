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
    logic [31:0] result3;
    logic [31:0] result4;
    logic complete;
    logic complete2;
    logic complete3;
    logic complete4;
    logic rst;
    logic mulhsu;
    logic mulhu;
    logic lui;
    logic mul;

    mulhsu mulhsu1 (
        .clk (clk),
        .x (x),
        .y (y),
        .mulhsu (mulhsu),
        .result (result),
        .complete (complete),
        .rst (rst)
    );

    mulhu mulhu1 (
        .clk (clk),
        .x (x),
        .y (y),
        .mulhu (mulhu),
        .result4 (result4),
        .complete4 (complete4),
        .rst (rst)
    );

    adder adder (
        .clk (clk),
        .read_data (read_data),
        .complete (complete),
        .complete2 (complete2),
        .complete3 (complete3),
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

    mul mul1 (
        .clk (clk),
        .x (x),
        .y (y),
        .mul (mul),
        .result3 (result3),
        .complete3 (complete3),
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
        .result3 (result3),
        .rst (rst),
        .mulhsu (mulhsu),
        .mulhu (mulhu),
        .lui (lui),
        .mul (mul)
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