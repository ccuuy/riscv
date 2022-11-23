module ALUMux(
    input ALUSrc,//0时ReadData2
    input [31:0]ReadData2,
    input [31:0]imm,
    output [31:0]ALU_input2
    );

    assign ALU_input2 = ALUSrc ? imm : ReadData2;


endmodule