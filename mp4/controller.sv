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

    // State machine for load instructions
    typedef enum logic [1:0] {
        NORMAL,     // Normal operation
        LOAD_WAIT   // Waiting for load data
    } state_t;
    
    state_t state;  // Current state
    logic [4:0] load_rd;  // Destination register for load instructions
    
    // create a test placeholder to debug whether the register has the proper value
    logic [31:0] test;

    logic [31:0] debug;

    logic debug2;


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

        // Initialize state machine to NORMAL state
        state = NORMAL;
    end


    // begin out clock loop
    always_ff @(posedge clk) begin
        // Reset write_mem every cycle (will be set to 1 if needed)
        write_mem <= 1'b0;
        
        // // State machine for instruction execution
        // case (state)
        //     NORMAL: begin
                // as long as the read_data is not equal to all zeros, continue executing the program
                if (read_data != stop) begin

                    // first read the last 7 bits to determine instruction type
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

                                    debug <= registers[read_data[19:15]];

                                    debug2 <= (registers[read_data[19:15]] < immediate);

                                    if ($signed(registers[read_data[19:15]]) < $signed(immediate)) begin
                                        registers[read_data[11:7]] <= registers[read_data[19:15]];
                                    end else begin
                                        registers[read_data[11:7]] <= 32'b0;
                                    end

                                end
                            endcase
                        end

                        // // Load instructions (lb, lh, lw, lbu, lhu)
                        // 7'b0000011: begin
                        //     // Calculate effective address
                        //     logic [31:0] effective_address;
                        //     // Sign extend the immediate
                        //     immediate = {{21{read_data[31]}}, read_data[30:20]};
                        //     effective_address = registers[read_data[19:15]] + immediate;
                            
                        //     // Set read address to the calculated effective address
                        //     read_address <= effective_address;
                            
                        //     // Store destination register for when data returns
                        //     load_rd <= read_data[11:7];
                            
                        //     // Set funct3 for memory operation based on instruction
                        //     funct3 <= read_data[14:12];
                            
                        //     // Change state to wait for memory read to complete
                        //     state <= LOAD_WAIT;
                        // end
                        
                        // Branch instructions (beq, bne, blt, bge, bltu, bgeu)
                        7'b1100011: begin
                            logic [31:0] branch_offset;
                            logic take_branch;
                            
                            // Sign-extend the immediate value for branch offset
                            // B-type immediate format: imm[12|10:5] = inst[31|30:25], imm[4:1|11] = inst[11:8|7]
                            branch_offset = {{20{read_data[31]}}, read_data[7], read_data[30:25], read_data[11:8], 1'b0};
                            
                            // Determine if branch should be taken based on funct3
                            case (read_data[14:12])
                                3'b000: begin // beq - branch if equal
                                    take_branch = (registers[read_data[19:15]] == registers[read_data[24:20]]);
                                end
                                
                                3'b001: begin // bne - branch if not equal
                                    take_branch = (registers[read_data[19:15]] != registers[read_data[24:20]]);
                                end
                                
                                3'b100: begin // blt - branch if less than (signed)
                                    take_branch = ($signed(registers[read_data[19:15]]) < $signed(registers[read_data[24:20]]));
                                end
                                
                                3'b101: begin // bge - branch if greater than or equal (signed)
                                    take_branch = ($signed(registers[read_data[19:15]]) >= $signed(registers[read_data[24:20]]));
                                end
                                
                                3'b110: begin // bltu - branch if less than (unsigned)
                                    take_branch = (registers[read_data[19:15]] < registers[read_data[24:20]]);
                                end
                                
                                3'b111: begin // bgeu - branch if greater than or equal (unsigned)
                                    take_branch = (registers[read_data[19:15]] >= registers[read_data[24:20]]);
                                end
                                
                                default: take_branch = 1'b0;
                            endcase
                            
                            // If branch taken, update PC (read_address)
                            if (take_branch) begin
                                read_address <= read_address + branch_offset - 4; // -4 because we add 4 at the end of this cycle
                            end
                        end
                        
                        // JALR - Jump and Link Register
                        7'b1100111: begin
                            // Save return address (PC+4) in rd
                            registers[read_data[11:7]] <= read_address + 4;
                            
                            // Calculate target address: rs1 + sign-extended immediate
                            immediate = {{21{read_data[31]}}, read_data[30:20]};
                            read_address <= (registers[read_data[19:15]] + immediate) & ~32'b1; // Clear lowest bit per spec
                            
                            // Skip the standard PC+4 increment at the end by subtracting 4
                            read_address <= read_address - 4;
                        end
                    endcase
                    // Move to the next instruction (PC+4) unless it's a load instruction
                    if (read_data[6:0] != 7'b0000011) begin
                        read_address <= read_address + 4; // moving the read address 32 bits
                    end
                end
            end
            
            // LOAD_WAIT: begin
            //     // Data from memory is now available in read_data
            //     // Write it to the destination register
            //     if (load_rd != 5'b00000) begin  // Don't write to x0
            //         registers[load_rd] <= read_data;
            //     end
                
            //     // Return to normal execution state
            //     state <= NORMAL;
                
            //     // Move to the next instruction
            //     read_address <= read_address + 4;
            // end
    //     endcase
    // end
    // assign test = registers['b01000];
endmodule