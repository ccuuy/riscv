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