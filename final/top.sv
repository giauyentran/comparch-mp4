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


    controller u0 (
        .clk     (clk),
        .read_data(read_data),
        .read_address (read_address),
        .write_address(write_address),
        .write_data(write_data),
        .write_mem(write_mem),
        .funct3(funct3)
    );

    memory #(
        .INIT_FILE      ("test")
    ) u1 (
        .clk            (clk), 
        .read_address   (read_address), 
        .read_data      (read_data),
        .write_address(write_address),
        .write_data(write_data),
        .write_mem(write_mem),
        .funct3(funct3)
    );

endmodule