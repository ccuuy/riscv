module RigisterWriteMux(
    input [31:0] DataFromRam,
    input MemToReg,
    input [31:0] ALU_result,
    output [31:0] DataToReg
    );

    assign DataToReg = MemToReg ? DataFromRam : ALU_result;
endmodule