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