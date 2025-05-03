`include "memory.sv"

module mulhsu
(
    input logic [31:0] x,
    input logic [31:0] y,
    input logic clk,
    input logic rst,
    input logic mulhsu,
    output logic [31:0] result,
    output logic complete
);

    logic [63:0] intermediate;

    initial begin
        complete <= 0;
    end


    // execute if x or y change (x and y are on the sensitivity list)

    always_ff @(posedge clk) begin
        if (mulhsu) begin
            intermediate <= x * y;
            complete <= 1;
        end
        if (rst) complete <= 0;
    end

    assign result = intermediate[63:32];

    // always @ (mulhsu) begin
    //     complete <= 0;
    //     intermediate <= x * y;
    // end

    // always @ (intermediate) begin
    //     result <= intermediate[63:32];
    //     complete <= 1;
        
    // end

    // always @ (rst) begin
    //     if (rst) complete <= 0;
    // end



endmodule

module lui (
    input logic [31:0] x,
    input logic rst,
    input logic clk,
    input logic lui,
    output logic [31:0] result2,
    output logic complete2
);

    initial begin
        complete2 <= 0;
    end

    // execute if x or y change (x and y are on the sensitivity list)

    always_ff @(posedge clk) begin
        if (lui) begin
        result2 <= x;
        complete2 <= 1;
        end
        if (rst) complete2 <= 0;
    end

    // always @ (lui) begin
    //     result2 <= x;
    //     complete2 <= 1;
    // end

    // always @ (rst) begin
    //     if (rst) complete2 <= 0;
    // end







endmodule



module adder
(
    input logic clk,
    input logic complete,
    input logic complete2,
    input logic [31:0] read_data,
    output logic [31:0] read_address
);

    int count = 0;

    initial begin

        // set the initial read_address to all zeros, first place in memory to read from
        read_address = 32'b0;

    end

    always_ff @(posedge clk) begin
        if (complete | complete2 | read_data == 32'b0) begin
            if (count > 1) begin
                count <= 0;
                read_address <= read_address + 4;
            end else count <= count + 1;
        end

    end

endmodule


module controller
(
    input logic clk,
    input logic  [31:0] read_data, // define the 32 bit command to read from memory module
    input logic complete,
    input logic complete2,
    input logic [31:0] result,
    input logic [31:0] result2,
    output logic write_mem,
    output logic [2:0] funct3,   
    output logic [31:0] write_address, 
    output logic [31:0] write_data, 
    output logic [31:0] x,
    output logic [31:0] y,
    output logic rst,
    output logic lui,
    output logic mulhsu
);


    // initialize the register array of 32 32-bit value arrays
    logic [31:0] registers [31:0];

    // define the stop condition of a 32 0-bit array which means there are no more values to read
    logic [31:0] stop = 32'b0;

    // define iterator object
    int i;

    // initialize branch logic
    logic take_branch;

    logic [31:0] debug;


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
        rst = 0;

    end


    // begin the loop
    always_ff @(posedge clk) begin
        // Reset write_mem every cycle (will be set to 1 if needed)
        write_mem <= 1'b0;
        
        if (read_data != stop) begin
            rst <= 0;
            

            // first read the last 7 bits to determine instruction type
            case (read_data[6:0])
                7'b0010011: begin // i-type path
                    

                    // read the 3 bits to determine what type of i-type operation it is
                    case (read_data[14:12])
                        3'b000: begin // perform addi 
                            
                            // assign the immediate value with sign extend
                            // add the immediate value with the corresponding register value to its destination register
                            x <= {{21{read_data[31]}}, read_data[30:20]};
                            y <= registers[read_data[19:15]];
                            registers[read_data[11:7]] <= result2;

                        end

                        3'b010: begin // perform slti 
                            // Place the value 1 in register rd if register rs1 is less than the signextended immediate, else 0 is placed
                            // assign the immediate value with 32 bits, but preserving the signed bit

                            if ($signed(registers[read_data[19:15]]) < $signed({{21{read_data[31]}}, read_data[30:20]})) begin
                                registers[read_data[11:7]] <= registers[read_data[19:15]];
                            end else begin
                                registers[read_data[11:7]] <= 32'b0;
                            end

                        end

                        3'b100: begin // perform xori
                            // Performs bitwise XOR on register rs1 and the sign-extended 12-bit immediate and place the result in rd

                            registers[read_data[11:7]] <= registers[read_data[19:15]] ^ {{21{read_data[31]}}, read_data[30:20]};

                        end

                        3'b110: begin // perform ori
                            // Performs bitwise OR on register rs1 and the sign-extended 12-bit immediate and place the result in rd

                            registers[read_data[11:7]] <= registers[read_data[19:15]] | {{21{read_data[31]}}, read_data[30:20]};

                        end


                        3'b111: begin // perform andi
                            // Performs bitwise AND on register rs1 and the sign-extended 12-bit immediate and place the result in rd

                            registers[read_data[11:7]] <= registers[read_data[19:15]] & {{21{read_data[31]}}, read_data[30:20]};

                        end

                        3'b011: begin // perform sltiu
                             // Place the value 1 in register rd if register rs1 is less than the immediate when both are 
                             // treated as unsigned numbers, else 0 is written to rd.

                            if (registers[read_data[19:15]] < {20'b0, read_data[31:20]}) begin
                                registers[read_data[11:7]] <= registers[read_data[19:15]];
                            end else begin
                                registers[read_data[11:7]] <= 32'b0;
                            end

                        end

                        3'b001: begin // perform slli
                            // Performs logical left shift on the value in register rs1 by the shift amount held in the lower 5 bits of the immediate

                            registers[read_data[11:7]] <= registers[read_data[19:15]] << read_data[24:20];

                        end

                        3'b101: begin // perform srli or srai

                            case (read_data[31:27])
                                5'b00000: begin // logical right shift
                                    // Performs logical right shift on the value in register rs1 by the shift amount held in the lower 5 bits of the immediat

                                    registers[read_data[11:7]] <= registers[read_data[19:15]] >> read_data[24:20];
                                end

                                5'b01000: begin // arithmetic right shift
                                    registers[read_data[11:7]] <= registers[read_data[19:15]] >>> read_data[24:20];

                                end

                            

                            endcase

                        end

                        // 3'b101: begin // perform srai
                        //     // Performs arithmetic right shift on the value in register rs1 by the shift amount held in the lower 5 bits of the immediate

                        //     registers[read_data[11:7]] <= registers[read_data[19:15]] >>> read_data[24:20];

                        // end


                    endcase
                end

                // 7'b1100111: begin // JALR - Jump and Link Register
                //     // Save return address (PC+4) in rd
                //     registers[read_data[11:7]] <= read_address + 4;

                //     // pc=(x[rs1]+sext(offset))&∼1
                //     read_address <= (registers[read_data[19:15]] + {{21{read_data[31]}}, read_data[30:20]}) & ~32'b1; // Clear lowest bit per spec
                    
                //     // Skip the standard PC+4 increment at the end by subtracting 4
                //     read_address <= read_address - 4;
                // end

               // I-TYPE Load instructions (lb, lh, lw, lbu, lhu)
               7'b0000011: begin
                   case(read_data[14:12])
                       3'b000: begin // lb - load half byte and sign extend to 32 and store in rd
                           registers[read_data[11:7]] <= $signed(read_data[7:0]);
                       end
                       3'b001: begin // lh - load 16-bit and sign extend to 32 and store in rd
                           registers[read_data[11:7]] <= $signed(read_data[15:0]);
                       end
                       3'b010: begin // lw - load 32-bit and sign extend to 32 and store in rd
                           registers[read_data[11:7]] <= read_data[7:0];
                       end
                       3'b100: begin // lbu - load 8-bit and sign extend to 32 and store in rd
                           registers[read_data[11:7]] <= {{24{1'b0}},read_data[7:0]};
                       end
                       3'b101: begin // lhu - load 16-bit and sign extend to 32 and store in rd
                           registers[read_data[11:7]] <= {{16{1'b0}},read_data[15:0]};
                       end
                   endcase
               end
                
                // Branch instructions (beq, bne, blt, bge, bltu, bgeu)
                7'b1100011: begin
                    
                    
                    // Sign-extend the immediate value for branch offset
                    // B-type immediate format: imm[12|10:5] = inst[31|30:25], imm[4:1|11] = inst[11:8|7]
                    
                    // Determine if branch should be taken based on funct3
                    case (read_data[14:12])
                        3'b000: begin // beq - branch if equal
                            take_branch <= (registers[read_data[19:15]] == registers[read_data[24:20]]);
                        end
                        
                        3'b001: begin // bne - branch if not equal
                            take_branch <= (registers[read_data[19:15]] != registers[read_data[24:20]]);
                        end
                        
                        3'b100: begin // blt - branch if less than (signed)
                            take_branch <= ($signed(registers[read_data[19:15]]) < $signed(registers[read_data[24:20]]));
                        end
                        
                        3'b101: begin // bge - branch if greater than or equal (signed)
                            take_branch <= ($signed(registers[read_data[19:15]]) >= $signed(registers[read_data[24:20]]));
                        end
                        
                        3'b110: begin // bltu - branch if less than (unsigned)
                            take_branch <= (registers[read_data[19:15]] < registers[read_data[24:20]]);
                        end
                        
                        3'b111: begin // bgeu - branch if greater than or equal (unsigned)
                            take_branch <= (registers[read_data[19:15]] >= registers[read_data[24:20]]);
                        end
                        
                        default: take_branch <= 1'b0;
                    endcase
                    
                    // // If branch taken, update PC
                    // if (take_branch) begin
                    //     read_address <= read_address + {{20{read_data[31]}}, read_data[7], read_data[30:25], read_data[11:8], 1'b0} - 4; // -4 because we add 4 at the end of this cycle
                    // end
                end
                
                // R-type instructions
                7'b0110011: begin
                    case(read_data[31:25])

                    7'b0000001: begin
                        case(read_data[14:12]) 

                            3'b000: begin
                                registers[read_data[11:7]] <= $signed(registers[read_data[24:20]]) * $signed(registers[read_data[19:15]]);
                            end
                            3'b011: begin
                                mulhsu <= 1;
                                y <= registers[read_data[24:20]];
                                x <= registers[read_data[19:15]];
                                registers[read_data[11:7]] <= result;

                                
                            end


                        endcase


                    end

                    default: begin
                        // funct3 cases
                        case (read_data[14:12])
                            3'b000: begin     
                                case(read_data[31:25])
                                    7'b0000000: begin
                                        registers[read_data[11:7]] <= registers[read_data[19:15]] + registers[read_data[24:20]];
                                    end
                                    7'b0100000: begin
                                        registers[read_data[11:7]] <= registers[read_data[19:15]] - registers[read_data[24:20]];
                                    end
                                endcase
                            end
                            
                            3'b001: begin // sll
                                registers[read_data[11:7]] <= registers[read_data[19:15]] << registers[read_data[24:20]][4:0];
                            end

                            3'b010: begin // slt
                                registers[read_data[11:7]] <= ($signed(registers[read_data[19:15]]) < $signed(registers[read_data[24:20]])) ? 32'd1 : 32'd0;
                            end

                            3'b011: begin // sltu
                                registers[read_data[11:7]] <= (registers[read_data[19:15]] < registers[read_data[24:20]]) ? 32'd1 : 32'd0;
                            end

                            3'b100: begin // xor
                                registers[read_data[11:7]] <= registers[read_data[19:15]] ^ registers[read_data[24:20]];
                            end

                            3'b101: begin
                                if (read_data[31:25] == 7'b0000000) begin
                                    // srl
                                    registers[read_data[11:7]] <= registers[read_data[19:15]] >> registers[read_data[24:20]][4:0];
                                end else if (read_data[31:25] == 7'b0100000) begin
                                    // sra
                                    registers[read_data[11:7]] <= $signed(registers[read_data[19:15]]) >>> registers[read_data[24:20]][4:0];
                                end
                            end

                            3'b110: begin // or
                                registers[read_data[11:7]] <= registers[read_data[19:15]] | registers[read_data[24:20]];
                            end

                            3'b111: begin // and
                                registers[read_data[11:7]] <= registers[read_data[19:15]] & registers[read_data[24:20]];
                            end
                        
                
                        endcase
                    end
                    endcase
                end



                7'b0110111: begin // lui
                    lui <= 1;
                    // U-type format: [31:12] immediate, [11:7] rd
                    x <= {12'b0, read_data[31:12]};
                    registers[read_data[11:7]] <= result2;
                end

                // 7'b0010111: begin // auipc
                //     // U-type: immediate << 12 + PC, result stored in rd
                //     registers[read_data[11:7]] <= read_address + {read_data[31:12], 12'b0};
                // end

                // 7'b1101111: begin // jal
                //     // J-type immediate: [31], [19:12], [20], [30:21], 0

                //     // Save return address (next instruction address) in rd
                //     registers[read_data[11:7]] <= read_address + 4;

                //     // Jump to target address (read_address + immediate)
                //     read_address <= read_address + {{12{read_data[31]}}, read_data[19:12], read_data[20], read_data[30:21], 1'b0} - 4; // minus 4 to cancel +4 at end of cycle
                // end

            endcase
            // // Move to the next instruction (PC+4) unless it's a load instruction
            // if (read_data[6:0] != 7'b0000011) begin
            //         // moving the read address 32 bits

            // end 
        end
        else begin
            rst <= 1;
            mulhsu <= 0;
            lui <= 0;
        end
    end


    assign debug = registers[4'b0011];
endmodule
