`define		lui				7'b0110111
`define		auipc			7'b0010111
`define		jal				7'b1101111
`define		jalr			7'b1100111
`define		B_type			7'b1100011
`define		load			7'b0000011
`define		store			7'b0100011
`define		I_type			7'b0010011
`define		R_type			7'b0110011

module ImmGen(
	input [31:0]instr,
	output [31:0]imme
	);
	 
	wire I;
	wire U;
	wire J;
	wire B;
	wire S;
	
	wire [31:0]I_imme;
	wire [31:0]U_imme;
	wire [31:0]J_imme;
	wire [31:0]B_imme;
	wire [31:0]S_imme;
	
	assign I=(instr[6:0]==`jalr) | (instr[6:0]==`load) | (instr[6:0]==`I_type);
	assign U=(instr[6:0]==`lui) | (instr[6:0]==`auipc);
	assign J=(instr[6:0]==`jal);
	assign B=(instr[6:0]==`B_type);
	assign S=(instr[6:0]==`store);
	
	//立即数符号扩展
	assign I_imme = {{20{instr[31]}},instr[31:20]}; 
	assign U_imme = {instr[31:12],{12{1'b0}}};
	assign J_imme = {{12{instr[31]}},instr[19:12],instr[20],instr[30:21],1'b0};   
	assign B_imme = {{20{instr[31]}},instr[7],instr[30:25],instr[11:8],1'b0};
	assign S_imme = {{20{instr[31]}},instr[31:25],instr[11:7]}; 
	
	assign imme = I ? I_imme : U ? U_imme : J ? J_imme : B ? B_imme : S ? S_imme : 32'd0;


endmodule
