module controller(
    input logic         clk,
    input logic  [31:0] instruction,
    output logic        write_mem,
    output logic [2:0]  funct3,
    output logic [31:0] write_address,
    output logic [31:0] write_data,
    output logic [31:0] read_address
);
    // RISC-V instruction fields
    logic [6:0] opcode;
    logic [4:0] rd, rs1, rs2;
    logic [2:0] funct3_instr;
    logic [6:0] funct7;
    logic [31:0] imm_u;
    
    // Register file
    logic [31:0] registers [0:31];
    
    // Program counter
    logic [31:0] pc = 32'd0;
    
    // Instruction decode
    assign opcode = instruction[6:0];
    assign rd = instruction[11:7];
    assign funct3_instr = instruction[14:12];
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign funct7 = instruction[31:25];
    assign imm_u = {instruction[31:12], 12'b0}; // U-type immediate
    
    // Initialize registers to 0
    initial begin
        for (int i = 0; i < 32; i++) begin
            registers[i] = 32'd0;
        end
    end
    
    // Memory access outputs
    assign write_mem = 1'b0; // We're not writing to memory in this implementation
    assign funct3 = 3'b010;  // Word access
    assign write_address = 32'd0;
    assign write_data = 32'd0;
    assign read_address = pc;  // Read from program counter address
    
    // Execute instruction
    always_ff @(posedge clk) begin
        case (opcode)
            7'b0110111: begin // LUI
                if (rd != 0) registers[rd] <= imm_u;
                pc <= pc + 4;
            end
            
            7'b0110011: begin // R-type instructions
                if (funct7 == 7'b0000001 && funct3_instr == 3'b001) begin // MULH
                    if (rd != 0) begin
                        // Sign-extended multiplication, take upper 32 bits
                        logic signed [63:0] product;
                        logic signed [31:0] a, b;
                        logic signed [31:0] result;
                        a = registers[rs1];
                        b = registers[rs2];
                        product = a * b;
                        result = product[63:32];
                        registers[rd] <= product[63:32];
                    end
                end
                pc <= pc + 4;
            end
            
            default: begin
                pc <= pc + 4; // Skip unknown instructions
            end
        endcase
    end
    
endmodule