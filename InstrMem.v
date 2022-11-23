module InstrMem(
	input [7:0]addr,
	output [31:0]instr
    );
	
	reg[31:0] rom[255:0];
	
    //rom进行初始化
    initial begin
        
        rom[0] = 32'b00000000001000000000000100010011;
        rom[1] = 32'b00000000001000000010000000100011;
        rom[2] = 32'b00000000000000000010000010000011;

    end
	
    assign instr = rom[addr];

endmodule
	
	
	
	