module Control(
    input [6:0] opcode,
    output reg ALUSrc, // 0时来自寄存器，1时来自立即数
    output reg[1:0] Branch,
    output reg MemRead,
    output reg MemtoReg, // 1 时写入dataMem中的, 0 时写ALU里的
    output reg MemWrite,
    output reg RegWrite,
    output reg ALUop // 1时结合func3进行判断，0时直接加
);


always@(*) begin
    case(opcode) 
        7'b0110011: begin // R型指令
            Branch = 2'b00;
            ALUSrc = 0;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            RegWrite = 1;
            ALUop = 1;
       end
        7'b0010011: begin // I型指令_立即数
            Branch = 2'b00;
            ALUSrc = 1;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            RegWrite = 1;
            ALUop = 1;
       end
        7'b0000011: begin// I型指令_load
            Branch = 2'b00;
            ALUSrc = 1;
            MemRead = 1;
            MemtoReg = 1;
            MemWrite = 0;
            RegWrite = 1;
            ALUop = 0;
        end
        7'b0100011: begin// S型指令
            Branch = 2'b00;
            ALUSrc = 1;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 1;
            RegWrite = 0;
            ALUop = 0;
        end
        7'b1100011: begin// B型指令
            Branch = 2'b10;
            ALUSrc = 1;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            RegWrite = 0;
            ALUop = 1;
        end
        7'b1101111: begin// J型指令
            Branch = 2'b01;
            ALUSrc = 1;
            MemRead = 0;
            MemtoReg = 0;
            MemWrite = 0;
            RegWrite = 1;
            ALUop = 0;
        end
    endcase
end                
endmodule