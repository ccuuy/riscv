module PCMux(
    input [7:0] PCin,
    input [7:0] imm,
    input branch,
    output [7:0] PCout
);

assign PCout = branch ? (PCin + imm) : (PCin + 8'b00000001);

endmodule
