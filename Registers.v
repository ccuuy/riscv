`define zero_word 32'b0
module Registers(
	input clk,
	input rst_n,
	input RegWrite, // 写使能
	input [4:0]ReadRegister1, 
	input [4:0]ReadRegister2,

	input [4:0]WriteRegister, // 写入地址
	input [31:0]WriteData, // 写入数据
	
	output [31:0]ReadData1,
	output [31:0]ReadData2
    );
	
	reg [4:0] regs [31:0];
	

	//read

	assign ReadData1=(ReadRegister1==5'd0)?32'b0: regs[ReadRegister1];
	assign ReadData2=(ReadRegister2==5'd0)?32'b0: regs[ReadRegister2];


	//write
		
	always@(posedge clk)
	begin
        if(!rst_n)
			begin
				regs[0]<=`zero_word;
				regs[1]<=`zero_word;
				regs[2]<=`zero_word;
				regs[3]<=`zero_word;
				regs[4]<=`zero_word;
				regs[5]<=`zero_word;
				regs[6]<=`zero_word;
				regs[7]<=`zero_word;
				regs[8]<=`zero_word;
				regs[9]<=`zero_word;
				regs[10]<=`zero_word;
				regs[11]<=`zero_word;
				regs[12]<=`zero_word;
				regs[13]<=`zero_word;
				regs[14]<=`zero_word;
				regs[15]<=`zero_word;
				regs[16]<=`zero_word;
				regs[17]<=`zero_word;
				regs[18]<=`zero_word;
				regs[19]<=`zero_word;
				regs[20]<=`zero_word;
				regs[21]<=`zero_word;
				regs[22]<=`zero_word;
				regs[23]<=`zero_word;
				regs[24]<=`zero_word;
				regs[25]<=`zero_word;
				regs[26]<=`zero_word;
				regs[27]<=`zero_word;
				regs[28]<=`zero_word;
				regs[29]<=`zero_word;
				regs[30]<=`zero_word;
				regs[31]<=`zero_word;
			end
		else if(RegWrite & (WriteRegister!=0))
			regs[WriteRegister]<=WriteData;	
	end
		

endmodule
