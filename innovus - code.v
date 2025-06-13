`timescale 1ns / 1ps

module innovus (start,clk1,count,I,read_i,read_d,A,B,L,S);      
input start,clk1;
input [31:0] I,L;
output [31:0] A,B;
output read_i,read_d;
output [15:0] count,S;

wire [2:0] operation;              
wire [2:0] rs1, rs2, rd;           
wire [6:0] opcode;                
wire [31:0] Z, acc, F;
wire ALUon;                        
wire mux_signal;                                                      
wire pc_signal;                                                
wire [0:7] FR;            
                                  
decoder DEC(.din(I), 
            .dout0(rs1), 
            .dout1(rs2), 
            .dout2(rd), 
            .dout3(opcode), 
            .dout4(S));

multiplexer MUX(.dout(F), 
                .din0(Z), 
                .din1(L), 
                .select(mux_signal), 
                .clk(clk1));
program_counter PC(.out(count), 
                   .in(S), 
                   .pc_signal(pc_signal), 
                   .clk(clk1),
                   .start(start));
register_bank RB(.destination_reg1_d(F), 
                 .destination_reg2_d(acc), 
                 .destination_reg3_d(S), 
                 .source_reg1_d(A), 
                 .source_reg2_d(B), 
                 .destination_reg_a(rd), 
                 .source_reg1_a(rs1), 
                 .source_reg2_a(rs2), 
                 .clk(clk1), 
                 .start(start),
                 .operation(operation));
arithmatic_logic_unit ALU(.dout0(Z), 
                          .FR(FR), 
                          .dout5(acc), 
                          .din0(A), 
                          .din1(B), 
                          .select(opcode), 
                          .clk(clk1), 
                          .enable(ALUon));
control_signals CS    (.read_i(read_i),                     
                       .ALUon(ALUon),
                       .read_d(read_d),
                       .mux_signal(mux_signal),                                            
                       .pc_signal(pc_signal),
                       .I(I),
                       .FR(FR),
                       .clk1(clk1),
                       .start(start),
                       .operation(operation));                          

endmodule

////////////////////////////////////////////////////////////////////////////

module decoder(din,
               dout0,
               dout1,
               dout2,
               dout3,
               dout4);
input [31:0] din;
output reg[2:0]  dout0, dout1, dout2;     
output reg[6:0]  dout3;
output reg[15:0] dout4;



always@(din)
begin
dout0 <= din[24:22];            
dout1 <= din[21:19];            
dout2 <= din[18:16];          
dout3 <= din[31:25];          
dout4 <= din[15:0];          
end

endmodule


//////////////////////////////////////////////////////////////////////////////


module program_counter(out,
                       in,
                       pc_signal,
                       clk,
                       start);     
input[15:0] in;                              
output reg[15:0] out;                  
input pc_signal, clk, start;            
reg [2:0] TEMP;                        
                                                                                   
always@(posedge clk)
begin

if(start == 1'b1) 
begin
out <= 16'h0000; 
TEMP <= 3'b001;
end

else
begin
if (TEMP == 3'b101)
begin
if      (pc_signal == 1'b0) begin out <= in;      TEMP <= 3'b001; end
else if (pc_signal == 1'b1) begin out <= out + 1; TEMP <= 3'b001; end
else                        begin out <= out;     TEMP <= 3'b001; end
end
else TEMP <= TEMP + 1;
end

end


endmodule

//////////////////////////////////////////////////////////////////////////////////

module arithmatic_logic_unit (dout0,
                              FR,
                              dout5,
                              din0,
                              din1,
                              select,
                              clk,
                              enable);
input [31:0] din0, din1;                                   
output reg[31:0] dout0, dout5;            
output reg [0:7] FR;                          
input [6:0] select;                      
input clk, enable;                             
                                               

always @(posedge clk)
begin
if(enable == 1'b1)
begin
if (select == 7'b0000001)         begin dout0 <= din0 + din1;                        FR <= 8'b01000000; end
else if(select == 7'b0000010)     begin dout0 <= din0 - din1;                        FR <= 8'b01000000; end
else if(select == 7'b0000011)     begin dout0 <= din0 & din1;                        FR <= 8'b01000000; end
else if(select == 7'b0000100)     begin dout0 <= din0 | din1;                        FR <= 8'b01000000; end
else if(select == 7'b0000101)     begin 
                                  if( din0 < din1 ) begin                            FR <= 8'b00010000; end   
                                  else              begin                            FR <= 8'b00000001; end   
                                  end
else if(select == 7'b0000110)     begin if( din0 > din1 ) begin                      FR <= 8'b00000001; end   
                                  else                    begin                      FR <= 8'b00010000; end
                                  end
else if(select == 7'b0000111)     begin                                              FR <= 8'b01000000; end
else if(select == 7'b0001000)     begin                                              FR <= 8'b00000000; end
else if(select == 7'b0001001)     begin dout0 <= dout0;  dout5 <= dout5;             FR <= FR;          end
else if(select == 7'b0001010)     begin dout0 <= (din0 >> 4)|(din0 << 32-4);         FR <= 8'b01000000; end
else if(select == 7'b0001011)     begin dout0 <= (din0 << 4)|(din0 >> 32-4);         FR <= 8'b01000000; end
else if(select == 7'b0001100)     begin 
                                  if(din0 == 0)                                begin FR <= 8'b00100000; end
                                  else                                         begin FR <= 8'b00000000; end
                                  end
else if(select == 7'b0001101)     begin 
                                  if(din0 != 0)                                begin FR <= 8'b00100000; end
                                  else                                         begin FR <= 8'b00000000; end
                                  end 
else if(select == 7'b0001110)     begin 
                                  if(din0 == din1)                             begin FR <= 8'b00100000; end
                                  else                                         begin FR <= 8'b00000000; end
                                  end 
else if(select == 7'b0001111)     begin 
                                  if(din0 != din1)                             begin FR <= 8'b00100000; end
                                  else                                         begin FR <= 8'b00000000; end
                                  end
else if(select == 7'b0010000)                                                  begin FR <= 8'b00100000; end
else if(select == 7'b0010001)     begin {dout5[31:0],dout0[31:0]} <= din0*din1;      FR <= 8'b00001000; end   
else if(select == 7'b0010010)     begin dout0[31:0] <= din0/din1;                    FR <= 8'b01000000; end
else if(select == 7'b0010011)     begin dout0[31:0] <= din0%din1;                    FR <= 8'b01000000; end
else if(select == 7'b0010100)     begin dout0 <= ~din0;                              FR <= 8'b01000000; end
else if(select == 7'b0010101)     begin dout0 <= din0;                               FR <= 8'b01000000; end
else if(select == 7'b0010110)     begin 
                                  if(FR[7] == 1'b1)                            begin FR <= 8'b00100001; end 
                                  else                                         begin FR <= 8'b00010000; end 
                                  end
else if(select == 7'b0010111)     begin 
                                  if(FR[3] == 1'b1)                            begin FR <= 8'b00110000; end 
                                  else                                         begin FR <= 8'b00000001; end 
                                  end  
else if(select == 7'b0011000)                                                  begin FR <= 8'b00000100; end 
else if(select == 7'b0011001)                                                  begin FR <= 8'b00000010; end               
else                              begin dout0 <= dout0;                              FR <= FR;          end  

end
end
endmodule


       
///////////////////////////////////////////////////////////////////////////
       
module multiplexer(dout, 
                   din0, 
                   din1, 
                   select, 
                   clk);
input [31:0] din0, din1;                
output reg [31:0] dout;                 
input select, clk;                  
                                     

always@ (posedge clk)
begin
if (select == 1'b0) dout <= din0;
else if (select == 1'b1) dout <= din1;
else dout <= dout;
end

endmodule

/////////////////////////////////////////////////////////////////////////////

module register_bank (destination_reg1_d, 
                      destination_reg2_d, 
                      destination_reg3_d, 
                      source_reg1_d, 
                      source_reg2_d, 
                      destination_reg_a, 
                      source_reg1_a, 
                      source_reg2_a, 
                      clk, 
                      start,
                      operation);

output reg [31:0] source_reg1_d, source_reg2_d; 
input [31:0] destination_reg1_d, destination_reg2_d; 
input [15:0] destination_reg3_d;
input [2:0] destination_reg_a, source_reg1_a, source_reg2_a;
input [2:0] operation;
input clk, start; 
reg [31:0] regbank[7:0];                                              
reg [31:0] TEMP;
integer i;


always @ (posedge clk)
begin 
   if(start == 1'b1)   
   for(i=0; i<8; i=i+1) regbank [i] <= 32'h00000000; 
   else
   begin 
   if (operation == 3'b000) 
           begin      
           regbank[destination_reg_a] <= destination_reg1_d;  
           end          
   else if (operation == 3'b001)
           begin       
           regbank[destination_reg_a]     <= destination_reg1_d;  
           regbank[destination_reg_a + 1] <= destination_reg2_d;  
           end             
   else if (operation == 3'b010)
           begin
           TEMP = regbank[destination_reg_a];
           TEMP = {destination_reg3_d , TEMP[15:0]};       
           regbank[destination_reg_a] = TEMP;                  
           TEMP = 32'h00000000;
           end  
   else if (operation == 3'b011) 
           begin  
           TEMP = regbank[destination_reg_a];
           TEMP = {TEMP[31:16] , destination_reg3_d};
           regbank[destination_reg_a] = TEMP;                   
           TEMP = 32'h00000000;
           end  
   else if (operation == 3'b100)  
           begin
           source_reg1_d <= source_reg1_d;   
           source_reg2_d <= source_reg2_d;
           end 
   else if (operation == 3'b101) 
           begin       
           source_reg1_d <= regbank[source_reg1_a]; 
           source_reg2_d <= regbank[source_reg2_a];
           end        
   else    begin end  
   end
end



endmodule

///////////////////////////////////////////////////////////////////////////////

module control_signals(read_i,                      
                       ALUon,
                       read_d,
                       mux_signal,                                              
                       pc_signal,
                       I,
                       FR,
                       clk1,
                       start,
                       operation);

output reg read_i,ALUon,read_d,mux_signal,pc_signal;
output reg [2:0] operation;
input [31:0] I;
input clk1,start;
input [0:7] FR;
reg [2:0] state;
         
always @(posedge clk1)
begin
if(start == 1'b1)                      state <= 3'b001;
else                          
begin

if(state == 3'b000) 
         begin
         if(start == 1'b1)             state <= 3'b001;
         else if(start == 1'b0)        state <= 3'b000;
         else                          state <= 3'b000;
         end
else if(state == 3'b001)               state <= 3'b010;
else if(state == 3'b010)               state <= 3'b011;
else if(state == 3'b011)               state <= 3'b100;
else if(state == 3'b100)               state <= 3'b101;
else if(state == 3'b101)
         begin      
         if ( I == 32'h12000000 )      state <= 3'b000;
         else                          state <= 3'b001;
         end
else state <= 3'b000;
end
end



always @(state)
begin

if(state == 3'b000)
    begin  
    read_i <= 0; 
    ALUon <= 0; 
    read_d <= read_d;
    mux_signal <= mux_signal;
    operation = 3'b100;     
    pc_signal <= pc_signal;                 
    end
    
else if(state == 3'b001)
    begin  
    read_i <= 1; 
    ALUon <= 0; 
    read_d <= read_d;
    mux_signal <= mux_signal;
    operation = 3'b100;
    pc_signal <= pc_signal;                   
    end
    
else if(state == 3'b010) 
    begin  
    read_i <= 0; 
    ALUon <= 0; 
    read_d <= read_d;
    mux_signal <= mux_signal;
    operation = 3'b101;
    pc_signal <= pc_signal;                     
    end

else if(state == 3'b011) 
    begin  
    read_i <= 0; 
    ALUon <= 1;     
    if(I[31:25] == 7'b0000111)       read_d <= 1'b1;           
    else if(I[31:25] == 7'b0001000)  read_d <= 1'b0;           
    mux_signal <= mux_signal;
    operation = 3'b100;
    pc_signal <= pc_signal;                     
    end

else if(state == 3'b100) 
    begin  
    read_i <= 0; 
    ALUon <= 0; 
    read_d <= read_d;
    if(I[31:25] == 7'b0000111) mux_signal <= 1'b1;           
    else                       mux_signal <= 1'b0;
    operation = 3'b100;
    pc_signal <= pc_signal;                    
    end
    
else if(state == 3'b101) 
    begin  
    read_i <= 0; 
    ALUon <= 0; 
    read_d <= read_d;
    mux_signal <= mux_signal;
    if(FR[2] == 0)      begin pc_signal <= 1'b1; end
    else if(FR[2] == 1) begin pc_signal <= 1'b0; end
    else                begin pc_signal <= pc_signal;  end   
    
    if(FR[1] == 1)       operation = 3'b000;
    else if(FR[4] == 1)  operation = 3'b001;
    else if(FR[5] == 1)  operation = 3'b010;
    else if(FR[6] == 1)  operation = 3'b011;                                        
    end

else     begin  
         read_i <= 0; 
         ALUon <= 0; 
         read_d <= read_d;
         mux_signal <= mux_signal;
         operation = 3'b100; 
         pc_signal <= pc_signal;                    
         end         
end

endmodule   
