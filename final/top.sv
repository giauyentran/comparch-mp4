`include "controller.sv"
`include "memory.sv"

module top(
    input logic clk
);
    // Signals between controller and memory
    logic write_mem;
    logic [2:0] funct3;
    logic [31:0] write_address;
    logic [31:0] write_data;
    logic [31:0] read_address;
    logic [31:0] read_data;
    logic led, red, green, blue;
    
    // Instantiate memory module
    memory #(
        .INIT_FILE("test.txt")
    ) mem (
        .clk(clk),
        .write_mem(write_mem),
        .funct3(funct3),
        .write_address(write_address),
        .write_data(write_data),
        .read_address(read_address),
        .read_data(read_data),
        .led(led),
        .red(red),
        .green(green),
        .blue(blue)
    );
    
    // Instantiate controller
    controller ctrl (
        .clk(clk),
        .instruction(read_data),
        .write_mem(write_mem),
        .funct3(funct3),
        .write_address(write_address),
        .write_data(write_data),
        .read_address(read_address)
    );
    
endmodule