module adder(X,Y,Cin,F,Cout);

  input X,Y,Cin;
  output F,Cout;

  assign F = X ^ Y ^ Cin;
  assign Cout = (X ^ Y) & Cin | X & Y;
endmodule