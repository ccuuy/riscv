[TOC]
# 简化单周期riscv处理器的实现
凭着感觉懵懵懂懂地居然真的独自完成了一个简化的riscv指令集处理器，虽说逻辑其实比较简单，但是由于实在没有系统学习过verilog，仿真的时候出现了好多bug，许多问题到最后虽然成功解决了，但还是没能搞清楚背后的原理，直到最后真的仿真通过的时候我甚至脑子都是懵的。把实现的过程在这里尽量具体地写一下，希望看到文章的人能够帮助我解答一些心中的疑惑，同时文章稍作修改后也会作为计算机组成原理实验报告的一部分上交。代码上传至<a href="https://github.com/ccuuy/riscv">riscv代码</a>

## 指令分析
最初想要实现完整的riscv32i指令集，但由于试验任务的时间要求以及我个人能力的限制，最终只能摘选了18个进行实现。有条件跳转指令只实现了beq，无符号跳转指令实现了jal(并不完整)，对于I型指令中的加载指令以及S型指令，则没有实现半字、字节级的操作和无符号的操作。
倘若之后有时间，会尝试实现完整的riscv32i指令集。

### 实现指令
| 指令      | 指令说明 |指令类型|opcode|func3|
| ----------- | ----------- | ----------- |----------- | ----------- |
| jal      | 无条件跳转      |J型指令|1101111|
|  beq  |  有条件跳转  |B型指令|1100011|**010**
|lw|加载字|I型指令|0000011|
|sw|存字|S型指令|0100011|
|add|加|R型指令|0110011|000
|sub|减|R型指令|0110011|**010**
|sll|逻辑左移|R型指令|0110011|001
|srl|逻辑右移|R型指令|0110011|011
|and|与|R型指令|0110011|110
|or|或|R型指令|0110011|110
|xor|异或|R型指令|0110011|100
|addi|加|I型指令|0010011|000
|subi|减|I型指令|0010011|**010**
|slli|逻辑左移|I型指令|0010011|001
|srli|逻辑右移|I型指令|0010011|011
|andi|与|I型指令|0010011|110
|ori|或|I型指令|0010011|110
|xori|异或|I型指令|0010011|100

出于简化指令集的简洁性考虑，在不破坏功能的前提下，我对sub指令以及beq指令实现的func3数据段进行了细微的调整，与原版riscv32i有出入，执行sub相关指令时需要手动修正机器码。
具体理由在后面提到。

## 处理器总体思路
为了能够之后可能进行的完整实现以及方便与课堂内容结合，我决定尽量完全按照黑皮书《计算机组成与设计 riscv版》进行实现
下图为书中数据通路实现
![](https://img2022.cnblogs.com/blog/2687686/202211/2687686-20221118231720933-836790656.png)
下面进行处理器实现时，所有的信号与模块都是与书中一一对应的，减轻了一些设计上的压力。
### 一些处理
整个处理器的核心在于控制模块，控制模块获取`opcode[6:0]`信号就可以发出控制其余所有模块的信号。
#### ALU与func3
为了简化操作，我们将ALU的操作码直接与指令的func3对应，在这个简化指令集中，这样实际上完全可行，但是问题是需要将sub和add进行区分，我们在这里实际上无需背离原版指令集，只要给ALUControl一个func7信号的一位让其区分add和sub即可，但是我选择不这么做，我认为区分add与sub的func3是个更优雅的选择。

### 分块拆解
直接实现未免过于困难，于是尝试先把每一个部分进行实现，最后整合成完整的数据通路。
下面列一下要实现的所有小模块。
|模块|说明|
|---|---|
InstrMem|指令存储器|
DataMem|数据存储器|
Rigisters|寄存器堆
PC|PC寄存器|
PCMux|PC多选器|
Control|主控制器
ALUMux|ALU来源多选器
ALU|ALU运算器
ALUControl|ALU运算控制
Branch_judge|判断指令是否跳转
ImmGen|立即数生成器
RigisterWriteMux|寄存器写入数据多选器

## 处理器完整实现
### 各分模块实现
接下来我们一步步实现每一个模块
#### InstrMem
类型|信号|注释
|---|---|---|
input|[7:0]addr|指令地址(总共可存${2^8}$个指令)|
output|[31:0]instr|地址对应的指令|

指令存储器相对比较容易实现，只要根据输入的地址取出相对应的指令即可。

代码实现
```verilog
module InstrMem(
	input [7:0]addr,
	output [31:0]instr
    );	
	reg[31:0] rom[255:0];	
    //rom进行初始化
    initial begin
        rom[0] = 32'b00000000000100000000000010010011;
        rom[1] = 32'b00000000000100001000000100110011;
        rom[2] = 32'b00000000001000010000000110110011;
        rom[3] = 32'b01000000000100011000000110110011;    
    end
	
    assign instr = rom[addr];

endmodule
```

#### DataMem
类型|信号|注释
|---|---|---|
input|clk|时钟信号|
input|rst_n|复位信号|
input|MemWrite|写使能信号|
input|MemRead|读使能信号|
input|[7:0]Address|数据存储器地址|
input|[31:0] WriteData|写入数据|
output|[31:0] ReadData|读出数据|

根据控制器处理得到的的读写使能信号，决定写入还是读出数据。
写入数据来自寄存器堆读出的第二个寄存器数据。
数据存储器地址来自主ALU运算。

代码实现：
```verilog
module DataMem(
	input clk,
	input rst_n,	
	input MemWrite,
	input MemRead,
	input [7:0]Address,
	input [31:0] WriteData,
	output [31:0] ReadData
    );
	
	
	reg [31:0]ram[255:0];
	assign ReadData = ram[Address];

	always@(posedge clk)
	begin
		if(!rst_n)// for循环不知道为什么会报错，仿真暂且这样初始化
			begin //对每一个存储器都置零
				ram[0]<=`zero_word;
				ram[1]<=`zero_word;
				ram[2]<=`zero_word;
				ram[3]<=`zero_word;
				ram[4]<=`zero_word;
				ram[5]<=`zero_word;
				ram[6]<=`zero_word;
			        ······//略去
			end
		else if(MemWrite)
			ram[Address]<=WriteData;	
	end

endmodule
```

#### Control
这里几乎是整个处理器的核心模块，既然是实验结束后复盘，可以先从这里写起，方便之后模块输入输出的描述。

类型|信号|注释
|---|---|---|
input|[6:0]opcode||
output|ALUSrc|决定ALU的第二个操作数来源|
output|[1:0] Branch|分辨beq, jal, 与其他指令。后面被输入到Branch_judge模块判断指令是否跳转|
output|MemWrite|数据存储器写使能信号|
output|MemRead|数据存储器读使能信号|
output|[7:0]Address|数据存储器地址|
output|[31:0] WriteData|写入数据|
output|[31:0] ReadData|读出数据|


代码实现：
```verilog
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
```

#### ImmGen
类型|信号|注释
|---|---|---|
input|[31:0]instr|指令|
output|[31:0]imme|立即数|

输入指令输出符号扩展之后的立即数，这个模块事实上是第一个我实现的模块，这时候我还抱着完成完整指令集实现的心态，把所有指令的立即数都进行了生成，然而毫无疑问后来失败跑路了，但这个模块见证了我曾经的雄心壮志（笑
```verilog
`define		lui			7'b0110111
`define		auipc			7'b0010111
`define		jal			7'b1101111
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
```
#### PC
事实上这两个模块我的实现遇到了未知的错误，在整合的数据通路中我需要保存当前的地址，我定义了wire类型的两个变量PCin和PCout，它们来存储经过多选器前后的地址，但是当我简单地`assign PCin = PCout;`时,令人难过的事情发生了。PC多选器变得一团糟，当最初的`PCin = 8'b0`被输入进去之后，输出的PCout信号竟然是`8'h00xx`未知态。经过长时间的debug，只是发现了将语句改为`assign PCin = instrAddress;//由PC产生的直接给入指令存储器的信号`时，仿真波形恢复了正常。
##### PCMUx
根据branch信息判断是否要进行跳转，跳转的话输出当前指令地址加立即数，不跳转则直接输出指令地址加一。
```verilog
module PC_mux(
    input [7:0] PCin,
    input [7:0] imm,
    input branch,
    output [7:0] PCout
);

assign PCout = branch ? (PCin + imm) : (PCin + 8'b00000001);

endmodule
```
##### PC
将输入地址输出，第一次或复位信息传来时输出指令地址为0。
```verilog
module PC(
    input clk,
    input rst_n,
    input [7:0] PCin,
    output reg [7:0] PCout
);
initial begin
    PCout = 0;
end

always@(posedge clk) begin
    if(!rst_n) PCout = 0;
    else PCout = PCin;
end

endmodule
```
#### ALU
##### ALUControl
这个模块用于生成ALU控制信号
```verilog
module ALUControl(
    input ALUop,
    input [2:0]func3,
    output [2:0]ALUControlOp
    );
	
    assign ALUControlOp = ALUop ? func3 : 3'b000;
endmodule
```
##### ALUMux
这个模块用于选择ALU输入信号来源
```verilog
module ALUMux(
    input ALUSrc,//0时ReadData2
    input [31:0]ReadData2,
    input [31:0]imm,
    output [31:0]ALU_input2
    );

    assign ALU_input2 = ALUSrc ? imm : ReadData2;


endmodule
```
##### ALU
这是ALU运算模块，后面会有快速加法的改进。
```verilog
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
```
#### Rigisters
##### RigisterWithMux
选择寄存器写入信息的来源。
```verilog
module RigisterWriteMux(
    input [31:0] DataFromRam,
    input MemToReg,
    input [31:0] ALU_result,
    output [31:0] DataToReg
    );

    assign DataToReg = MemToReg ? DataFromRam : ALU_result;
endmodule
```
##### Rigisters
这是寄存器模块。
为了仿真成功先为每一寄存器赋值0。
```verilog
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

```
#### BranchJudge
通过控制器生成的isbranch和ALU运算得到的Zero判断是否跳转
```verilog
module BranchJudge(
    input [1:0]Branch,
    input Zero,
    output reg isBranch // 
    );

wire beq;
wire jal;
wire others;
assign beq = (Branch == 2'b10) ? 1 : 0;
assign jal = (Branch == 2'b01) ? 1 : 0;
assign others = (Branch == 2'b00) ? 1 : 0;
always@(*)
begin
    if(others) isBranch = 1'b0;
    else if(jal) isBranch = 1'b1;
    else if(beq) isBranch = Zero;
end


endmodule
```
### 模块整合
这里我们将完整的CPU分成两个存储器与其他部件组成的数据通路
#### DataPath
```verilog
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
```
#### riscvWithMem
```verilog
module RiscvWithMem(
    input clk,
	input rst_n,
    output [31:0] OutIO
    );

    wire [7:0]InstrAddress;
    wire [31:0]NextInstr;

    wire MemWrite;
	wire MemRead;
	wire [7:0] DataAddress;
	wire [31:0] WriteData;
	wire [31:0] ReadData;


    DataPath DataPath(clk, rst_n, NextInstr, ReadData, MemWrite, MemRead, DataAddress, WriteData, InstrAddress);

    DataMem DataMem(clk, rst_n, MemWrite, MemRead, DataAddress, WriteData, ReadData, OutIO);

    InstrMem InstrMem(InstrAddress, NextInstr);
endmodule
```
## 存在的问题
### jal
我没有理解这个命令如何在数据通路中实现实现，PC地址+4怎样被存放到寄存器中，事实上我没有对这个进行处理，这个指令并不完善。
### 控制模块
由于目前实现命令较少，没有对逻辑进行简化，而只使用了几个case对不同类型的指令发出不同的控制信号。
## 仿真检验
接下来我们对指令进行仿真执行，看一下我们的cpu是否可以正常运行。
```verilog
`timescale 1ns / 1ps


module test;

	// Inputs
	reg clk;
	reg rst_n;

	RiscvWithMem rrrrrrr (
		.clk(clk), 
		.rst_n(rst_n)
	);
    always #5 clk= ~clk;
	initial begin
		clk = 0;
		rst_n = 0;

		#10;
		rst_n=1;
        

	end
      
endmodule
```
### add sub
下面是仿真代码。
```
addi x1,x0,1
add x2,x1,x1
add x3,x2,x2
sub x3,x3,x1

addi x4,x0,-4
add x5,x4,x4
sub x6,x5,x3
sub x7,x6,x4
add x8,x7,x3

00000000000100000000000010010011
00000000000100001000000100110011
00000000001000010000000110110011
01000000000100011000000110110011
11111111110000000000001000010011
00000000010000100000001010110011
01000000001100101000001100110011
01000000010000110000001110110011
00000000001100111000010000110011
```
![](https://img2022.cnblogs.com/blog/2687686/202211/2687686-20221122214910126-28635050.png)
仿真大成功
### jal beq
```
addi x1,x0,1
addi x2,x0,2
jal x31,label1
addi x3,x0,3
addi x4,x0,4
add x5,x2,x2
label1:
beq x4,x5,label1
addi x6,x0,6

00000000000100000000000010010011
00000000001000000000000100010011
00000000100000000000111111101111
00000000001100000000000110010011
00000000010000000000001000010011
00000000001000010000001010110011
00000000010100100010010001100011
00000000011000000000001100010011
```
![](https://img2022.cnblogs.com/blog/2687686/202211/2687686-20221122223820273-2089196395.png)
仿真第一列表示是否是跳转指令，第二列表示是否跳转，第三列是指令内容，我们的跳转指令也与预期相符。

### ld和sd指令
```
addi x2,x0,2
sw x2,0,x0
lw x1,0,x0

00000000001000000000000100010011
00000000001000000010000000100011
00000000000000000010000010000011
```
![](https://img2022.cnblogs.com/blog/2687686/202211/2687686-20221123192912839-386097196.png)
上图为register
下图为DataMem
![](https://img2022.cnblogs.com/blog/2687686/202211/2687686-20221123193002428-25685905.png)

## 改进
### 超前进位加法
```verilog
module adder(X,Y,Cin,F,Cout);

  input X,Y,Cin;
  output F,Cout;

  assign F = X ^ Y ^ Cin;
  assign Cout = (X ^ Y) & Cin | X & Y;
endmodule
```
```verilog
module adder_4(
      input [4:1] x,
      input [4:1] y,
      input c0,
      output c4,
      output Gm,
      output Pm,
      output [4:1] F
      );

      wire p1,p2,p3,p4,g1,g2,g3,g4;
      wire c1,c2,c3;
   

cla4 CLA(c0,c1,c2,c3,c4,p1,p2,p3,p4,g1,g2,g3,g4);

      adder adder1(
                    .X(x[1]),
                    .Y(y[1]),
                    .Cin(c0),
                    .F(F[1]),
                    .Cout()
                );

      adder adder2(
                    .X(x[2]),
                    .Y(y[2]),
                    .Cin(c1),
                    .F(F[2]),
                    .Cout()
                );  

      adder adder3(
                    .X(x[3]),
                    .Y(y[3]),
                    .Cin(c2),
                    .F(F[3]),
                    .Cout()
                );

      adder adder4(
                    .X(x[4]),
                    .Y(y[4]),
                    .Cin(c3),
                    .F(F[4]),
                    .Cout()
                );   

assign p1 = x[1] ^ y[1];     
assign p2 = x[2] ^ y[2];
assign p3 = x[3] ^ y[3];
assign p4 = x[4] ^ y[4];

assign g1 = x[1] & y[1],
assign g2 = x[2] & y[2],
assign g3 = x[3] & y[3],
assign g4 = x[4] & y[4];

assign Pm = p1 & p2 & p3 & p4,
assign Gm = g4 ^ (p4 & g3) ^ (p4 & p3 & g2) ^ (p4 & p3 & p2 & g1);

endmodule
```
```verilog
module adder32(A,B,S,C32);
     input [32:1] A;
     input [32:1] B;
     output [32:1] S;
     output C32;

     wire px1,gx1,px2,gx2;
     wire c16;

  CLA_16 CLA1(
        .A(A[16:1]),
        .B(B[16:1]),
        .c0(0),
        .S(S[16:1]),
        .px(px1),
        .gx(gx1)
    );

  CLA_16 CLA2(
        .A(A[32:17]),
        .B(B[32:17]),
        .c0(c16),
        .S(S[32:17]),
        .px(px2),
        .gx(gx2)
    );

  assign c16 = gx1 ^ (px1 && 0), //c0 = 0
         C32 = gx2 ^ (px2 && c16);

endmodule
```
```verilog
module cla4(c0,c1,c2,c3,c4,p1,p2,p3,p4,g1,g2,g3,g4);

     input c0,g1,g2,g3,g4,p1,p2,p3,p4;
     output c1,c2,c3,c4;

     assign c1 = g1 ^ (p1 & c0),
            c2 = g2 ^ (p2 & g1) ^ (p2 & p1 & c0),
            c3 = g3 ^ (p3 & g2) ^ (p3 & p2 & g1) ^ (p3 & p2 & p1 & c0),
            c4 = g4^(p4&g3)^(p4&p3&g2)^(p4&p3&p2&g1)^(p4&p3&p2&p1&c0);

endmodule
```
```verilog
module cla16(A,B,c0,S,px,gx);
    input [16:1] A;
    input [16:1] B;
    input c0;
    output gx,px;
    output [16:1] S;

    wire c4,c8,c12;
    wire Pm1,Gm1,Pm2,Gm2,Pm3,Gm3,Pm4,Gm4;

    adder_4 adder1(
        .x(A[4:1]),
        .y(B[4:1]),
        .c0(c0),
        .c4(),
        .F(S[4:1]),
        .Gm(Gm1),
        .Pm(Pm1)
    );

    adder_4 adder2(
        .x(A[8:5]),
        .y(B[8:5]),
        .c0(c4),
        .c4(),
        .F(S[8:5]),
        .Gm(Gm2),
        .Pm(Pm2)
    );

    adder_4 adder3(
        .x(A[12:9]),
        .y(B[12:9]),
        .c0(c8),
        .c4(),
        .F(S[12:9]),
        .Gm(Gm3),
        .Pm(Pm3)
    );

    adder_4 adder4(
        .x(A[16:13]),
        .y(B[16:13]),
        .c0(c12),
        .c4(),
        .F(S[16:13]),
        .Gm(Gm4),
        .Pm(Pm4)
    );

    assign c4 = Gm1 ^ (Pm1 & c0);
    assign c8 = Gm2 ^ (Pm2 & Gm1) ^ (Pm2 & Pm1 & c0);
    assign c12 = Gm3 ^ (Pm3 & Gm2) ^ (Pm3 & Pm2 & Gm1) ^ (Pm3 & Pm2 & Pm1 & c0);

    assign px = Pm1 & Pm2 & Pm3 & Pm4;
    assign gx = Gm4 ^ (Pm4 & Gm3) ^ (Pm4 & Pm3 & Gm2) ^ (Pm4 & Pm3 & Pm2 & Gm1);

endmodule
```
## 输入输出
做一个简单的输出，
在DataMem里加一个输出`output IO_out = ram[0];`，再放到最顶层模块的输出里，可以把输出设备绑到数据存储器0x00位置
具体实现已经包含在上面的代码中