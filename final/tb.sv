`timescale 10ns/10ns
`include "top.sv"

module top_tb;
    logic clk = 0;
    
    top u0(.clk(clk));
    
    // Test values for registers
    initial begin
        $dumpfile("top.vcd");
        $dumpvars(0, top_tb);
        
        // Monitor register values for validation
        $monitor("Time=%0t, PC=%0d, Reg[8]=%h, Reg[9]=%h, Reg[10]=%h", 
                 $time, u0.ctrl.pc, u0.ctrl.registers[8], u0.ctrl.registers[9], 
                 u0.ctrl.registers[10]);
        
        #2000
        $finish;
    end
    
    // Generate clock
    always begin
        #4 clk = ~clk;
    end
    
endmodule