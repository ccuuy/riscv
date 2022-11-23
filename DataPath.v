module DataPath(
    input clk,
	input rst_n,
    input [31:0]Instruction,
    input [31:0] DataFromRam,//从DataMem中取得的数据

    output MemWrite,
    output MemRead,
    output [7:0]DataAddress,
    output [31:0]DataToRam, //要写入到DataMem中的数据
    output [7:0]InstrAddress
    );
    
    wire branch; // 交给PC模块进行判断的branch
    wire [7:0] PCin;
    wire [7:0] PCout;

    wire Zero;//
    
    wire [31:0]imm; //32位立即数

    wire [6:0] opcode;
    wire ALUSrc;
    wire [1:0] Branch; //需要交给branch_judge结合Zero共同判断
    wire MemtoReg;
    wire RegWrite;
    wire ALUop;

	wire [31:0]ReadData1; // 寄存器读出数据
	wire [31:0]ReadData2;

    wire [2:0]ALUControlOp; // ALU_control通过ALUop对ALU的命令
    
    wire [31:0]ALU_result;

    wire [31:0] DataToReg;
    wire [31:0] ALU_input2;

    assign DataAddress = ALU_result[7:0];

    ImmGen ImmGen(Instruction, imm);
    BranchJudge BranchJudge(Branch, Zero, branch);

    assign PCin = InstrAddress;
    assign DataToRam = ReadData2;
    //PC操作
    PCMux PCMux(PCin, imm[7:0], branch, PCout);
    PC PC(clk, rst_n, PCout, InstrAddress);

    Control Control(Instruction[6:0], ALUSrc, Branch, MemRead, MemtoReg, MemWrite, RegWrite, ALUop);
    Registers Registers(clk, rst_n, RegWrite, Instruction[19:15], Instruction[24:20], Instruction[11:7], DataToReg, ReadData1, ReadData2);
    ALUMux ALUMux(ALUSrc, ReadData2, imm, ALU_input2);
    ALUControl ALUControl(ALUop, Instruction[14:12], ALUControlOp);
    //ALU计算
    ALU ALU1(ReadData1, ALU_input2, ALUControlOp, ALU_result, Zero);
    //beqALU计算
    RigisterWriteMux RigisterWriteMux(DataFromRam, MemtoReg, ALU_result, DataToReg);


endmodule