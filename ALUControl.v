module ALUControl(
    input ALUop,
    input [2:0]func3,
    output [2:0]ALUControlOp
    );
	
    assign ALUControlOp = ALUop ? func3 : 3'b000;
endmodule