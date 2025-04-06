`include "memory.sv"


module controller
(
    input logic clk,
    input logic  [31:0] read_data, // define the 32 bit command to read from memory module
    output logic write_mem,
    output logic [2:0] funct3,   
    output logic [31:0] write_address, 
    output logic [31:0] write_data, 
    output logic [31:0] read_address
);

    // initialize the register array of 32 32-bit value arrays
    logic [31:0] registers [31:0];

    // define the stop condition of a 32 0-bit array which means there are no more values to read
    logic [31:0] stop = 32'b0;

    // define iterator object
    int i;

    // create a placeholder for the immediate value
    logic [31:0] immediate;

    // create a test placeholder to debug whether the register has the proper value
    logic [31:0] test;


    initial begin
        //------initial setup, this where all of the outputs are defined-------

        // initialize the registers to all be zero at the beginning
        for (i = 0; i < 32; i++) begin
            registers[i] = 32'd0;
        end

        // set write_mem to 0 as we do not want to initially write anything to memory
        write_mem = 1'b0;

        // set funct3 to instruct the memory module to read the full 32 bits from the memory address
        funct3 = 3'b010;

        // initialize the write address and data to all 0;
        write_address = 32'b0;
        write_data = 32'b0;


        // set the initial read_address to all zeros, first place in memory to read from
        read_address = 32'b0;


    end


    // begin out clock loop
    always_ff @(posedge clk) begin
        // as long as the read_data is not equal to all zeros, continue excuting the program
        if (read_data != stop) begin

            // first read the last 7 bits to determine whether the instruction type is u,r, or i
            case (read_data[6:0])
                7'b0010011: begin // i-type path

                    // read the 3 bits to determine what type of i-type operation it is
                    case (read_data[14:12])
                        3'b000: begin // perform addi 
                            // assign the immediate value with 20 zeros in front
                            immediate <= {20'b0, read_data[31:20]};

                            // add the immediate value with the corresponding register value to its destination register
                            registers[read_data[11:7]] <= immediate + registers[read_data[19:15]];

                            
                        end

                        3'b010: begin // perform slti 
                            // Place the value 1 in register rd if register rs1 is less than the signextended immediate, else 0 is placed
                            // assign the immediate value with 32 bits, but preserving the signed bit
                            immediate <= {{20{read_data[31:30]}}, read_data[31:20]};

                            if ($signed(registers[read_data[19:15]]) < $signed(immediate)) begin
                                registers[read_data[11:7]] <= registers[read_data[19:15]];
                            end else begin
                                registers[read_data[11:7]] <= 32'b0;
                            end


                        end

                    endcase

                    






                end


            endcase

            



            
        

            
            // iterate the read address
            read_address <= read_address + 32; // moving the read address 32 bits
        end

        test <= registers['b00001];
    end







endmodule
