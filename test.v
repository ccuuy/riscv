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