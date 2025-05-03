`timescale 10ns/10ns
`include "top.sv"

module top_tb;

    logic clk = 0;
    logic [31:0] read_data; // define the 32 bit command to read from memory module
    logic write_mem;
    logic [2:0] funct3;   
    logic [31:0] write_address;
    logic [31:0] write_data;
    logic [31:0] read_address;


    top u0 (.clk (clk));


    //140000000
    initial begin
        $dumpfile("top.vcd");
        $dumpvars(0, top_tb);
        #2000
        $finish;
    end

    always begin
        #4
        clk = ~clk;
    end

endmodule