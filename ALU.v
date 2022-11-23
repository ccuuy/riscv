module ALU (
            input [31: 0]A,
            input [31: 0]B,
            input [2: 0]ALUop,
            output [31: 0]out,
            output Zero //Branch判断
        );

// 计算结果
wire [31: 0]addOut;
wire [31: 0]subOut;
wire [31: 0]andOut;
wire [31: 0]orOut;
wire [31: 0]xorOut;
wire [31: 0]sllOut;
wire [31: 0]srlOut;

 // 输出寄存器
reg [31: 0]         out_reg;

assign Zero = !out ? 1 : 0;

 // 计算输出
assign addOut[31: 0] = A[31: 0] + B[31: 0];
assign subOut[31: 0] = A[31: 0] - B[31: 0];
assign andOut[31: 0] = A[31: 0] & B[31: 0];
assign orOut[31: 0] = A[31: 0] | B[31: 0];
assign xorOut[31: 0] = A[31: 0] ^ B[31: 0];
assign sllOut[31: 0] = A[31: 0] << B[4: 0];
assign srlOut[31: 0] = A[31: 0] >> B[4: 0];
 
// 输出选择

assign out = out_reg;

always @(*) begin
    case (ALUop)
        3'b000:begin  // ADD
                out_reg[31: 0] = addOut[31: 0];
        end
        3'b010:begin  // SUB
                out_reg[31: 0] = subOut[31: 0];
        end
        3'b110:begin  // AND
                out_reg[31: 0] = andOut[31: 0];
        end
        3'b110:begin  // OR
                out_reg[31: 0] = orOut[31: 0];
        end
        3'b100:begin  // XOR
                out_reg[31: 0] = xorOut[31: 0];
        end
        3'b001:begin  // SLL
                out_reg[31: 0] = sllOut[31: 0];
        end
        3'b011:begin  // SRL
                out_reg[31: 0] = srlOut[31: 0];
        end
        default: begin 
            out_reg[31: 0] = 32'b0;
        end
    endcase
end

 endmodule