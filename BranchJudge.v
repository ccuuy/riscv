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
