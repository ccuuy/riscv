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