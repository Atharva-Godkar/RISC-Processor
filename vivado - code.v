`timescale 1ns / 1ps

module risc_processor (start,clk1);      // use pushbutton for start & fn. generator for clk1
input start,clk1;

wire [2:0] operation;              // operation = load + stor1 + store2. or read_r + write_ra + write_rb
wire [2:0] rs1, rs2, rd;           // rs1 & rs2 are source registers, rd is destination register
wire [6:0] opcode;                 // opcode tells arithmatic logic unit, which operation has to be performed
wire [15:0] S;                     // it has multiple uses, generally used as address for branch intruction
wire [15:0] count;                 // count, holds address of next instruction to be executed
wire [31:0] A, B, Z, F, L, acc;    // A,B is data loaded from register bank
                                   // Z is output of arithmetic logic unit
                                   // F is final data that will be put into register bank
                                   // L is the data loaded from RAM 
                                   // acc & Z are used together to store 64 bit product

//wire read_r;                     // read_r = 1 depending on rs1, rs2 values, data from RB is given to ALU
//wire write_ra;                   // write_ra = 1  reg[address] = F
                                   // write_ra = 0  reg[address] = F   reg[address+1] = acc
//wire write_rb;                   // write_rb = 1  reg[31:16] = address
                                   // write_rb = 0  reg[15:0] = address
wire read_i;                       // read_i = 1 ROM send next instruction to decoder
wire ALUon;                        // ALUon = 1 ALU performs operation on data recieved from RB
wire read_d;                       // read_d = 1 read data from RAM, read_d = 0 write data into RAM
wire mux_signal;                   // mux_signal = 1 data from RAM gets stored into RB
                                   // mux_signal = 0 data from ALU gets stored into RB
wire pc_signal;                    // pc_signal = 1 increment current address
                                   // pc_signal = 0 load new address
wire [31:0] I;                     // 32 bit instruction = | opcode(7) | rs1(3) | rs2(3) | rd(3) | address(16) | 
wire [0:7] FR;                     // 8 bit flag register
                                   // FR[1] = 1 for all intructions except MUL,MOVU,MOVL
                                   // FR[2] = 1 to load new address, FR[2] = 0 to increase address
                                   // FR[3] = 1 for less than is true, rs1 < rs2
                                   // FR[4] = 1 for only MUL instruction
                                   // FR[5] = 1 for only MOVU instruction
                                   // FR[6] = 1 for only MOVL instruction
                                   // FR[7] = 1 for greater than is true, rs1 > rs2
                                   // FR[0] are unused;
          

read_only_memory ROM(.address(count), 
                     .dout(I), 
                     .clk(clk1), 
                     .read(read_i),
                     .start(start));
decoder DEC(.din(I), 
            .dout0(rs1), 
            .dout1(rs2), 
            .dout2(rd), 
            .dout3(opcode), 
            .dout4(S));
random_access_memory RAM(.address(S), 
                         .dout(L), 
                         .din0(B), 
                         .din1(A), 
                         .clk(clk1), 
                         .read(read_d),
                         .start(start));
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
dout0 <= din[24:22];            // opcode 7 bit
dout1 <= din[21:19];            // rs1 3 bit
dout2 <= din[18:16];            // rs2 3 bit
dout3 <= din[31:25];            // rd  3 bit
dout4 <= din[15:0];             // address 16 bit
end

endmodule


//////////////////////////////////////////////////////////////////////////////


module program_counter(out,
                       in,
                       pc_signal,
                       clk,
                       start);     
input[15:0] in;                              // 16 bit synchronous up counter
output reg[15:0] out;                        // counts from 0000 to FFFF
input pc_signal, clk, start;                 // counts from 0 to 65535
reg [2:0] TEMP;                              // to ensure PC works only between state 5 & stae 1
                                             // this neede cuz we removed 1'bx
                                            

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

module read_only_memory (address, 
                         dout, 
                         clk, 
                         read,
                         start);   
input[15:0] address;            // 16 bit address from program counter
output reg[31:0] dout;          // 32 bit instruction is output given by ROM
input clk,read,start;                 // when read = 1 & clk = posedge, then instruction is generated 
// reg [31:0] rom_mem [65535:0];   // 2^16 = 65536 = 64Kb of program memory (ROM)
reg [31:0] rom_mem [255:0];   // 2^8 = 256 bytes of program memory (ROM)
integer i;                      // to initialize all memory locations with zero values
                                // ROM has 65536 memory locations, each of size 32 bits


always@ (posedge clk) 
begin
if(start == 1'b1)
begin
for(i=0; i<255; i=i+1) rom_mem [i] = 32'h00000000; 
end
else
begin
if(read == 1'b1) dout <= rom_mem[address];
else             dout <= dout;
end
end

endmodule





//////////////////////////////////////////////////////////////////



module arithmatic_logic_unit (dout0,
                              FR,
                              dout5,
                              din0,
                              din1,
                              select,
                              clk,
                              enable);
input [31:0] din0, din1;                       // din0 = A     din1 = B               
output reg[31:0] dout0, dout5;                 // dout0 = Z    dout5 = acc   
output reg [0:7] FR;                           // FR is 8 bit flag register
input [6:0] select;                            // select = opcode
input clk, enable;                             // when enable = 1 & clk = posedge, ALU generates output
                                               

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
else                              begin dout0 <= dout0;                              FR <= FR;          end  /*FR <= 8'bxxxxxxxx;*/
// endcase
end
end
endmodule


       
///////////////////////////////////////////////////////////////////////////
       
module random_access_memory(address,
                            dout,
                            din0,
                            din1,
                            clk,
                            read,
                            start);
input [15:0] address;              // 16 bit address from decoder
input [31:0] din0,din1;            // din0 = B  din1 = A
output reg [31:0] dout;            // data stored in RAM given as output
input clk,read,start;              // read = 1 data is read from RAM, read = 0 data stored into RAM
// reg [31:0] ram_mem [65535:0];   // 2^16 = 65536 = 64Kb of data memory (RAM)
reg [31:0] ram_mem [255:0];        // 2^8 = 256 bytes of data memory (RAM)
integer i;                         // to initialize all memory location with zero
reg [2:0] TEMP;                    // to ensure RAM works only between state 3 & stae 4
                                   // this neede cuz we removed 1'bx 

always@(posedge clk)
if (start == 1'b1 || TEMP == 3'b101)       TEMP <= 3'b001;
else                                       TEMP <= TEMP + 1;

always@(posedge clk)
begin
if(start == 1'b1)  for(i=0; i<255; i=i+1) ram_mem [i] <= 32'h00000000;
else
begin
if (TEMP == 3'b011)
begin
if     (read == 1'b0) ram_mem[address + din1] <= din0; 
else if(read == 1'b1) dout <= ram_mem[address + din1]; 
end
end
end

endmodule

/////////////////////////////////////////////////////////////////////////

module multiplexer(dout, 
                   din0, 
                   din1, 
                   select, 
                   clk);
input [31:0] din0, din1;                // din0 = Z,  din1 = L
output reg [31:0] dout;                 // dout = F
input select, clk;                      // select = 1 then data from RAM gets stored into RB
                                        // select = 0 then data from ALU gets stored into RB

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
reg [31:0] regbank[7:0];  // 8 GPR each of size 32 bits                                             
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
           regbank[destination_reg_a] <= destination_reg1_d;  // rd <= Z (simple load operation)
           end          
   else if (operation == 3'b001)
           begin       
           regbank[destination_reg_a]     <= destination_reg1_d;  //rd <= Z (multiplication operation)
           regbank[destination_reg_a + 1] <= destination_reg2_d;  //rd + 1 <= acc
           end             
   else if (operation == 3'b010)
           begin
           TEMP = regbank[destination_reg_a];
           TEMP = {destination_reg3_d , TEMP[15:0]};       
           regbank[destination_reg_a] = TEMP;                    //rd <= S (MOVU operation)
           TEMP = 32'h00000000;
           end  
   else if (operation == 3'b011) 
           begin  
           TEMP = regbank[destination_reg_a];
           TEMP = {TEMP[31:16] , destination_reg3_d};
           regbank[destination_reg_a] = TEMP;                    //rd <= S (MOVL operation)
           TEMP = 32'h00000000;
           end  
   else if (operation == 3'b100)  
           begin
           source_reg1_d <= source_reg1_d;   //  no change in wire A wire B
           source_reg2_d <= source_reg2_d;
           end 
   else if (operation == 3'b101) 
           begin       
           source_reg1_d <= regbank[source_reg1_a];  //  simple store operation
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
    if(I[31:25] == 7'b0000111)       read_d <= 1'b1;           // load
    else if(I[31:25] == 7'b0001000)  read_d <= 1'b0;           // store
    mux_signal <= mux_signal;
    operation = 3'b100;
    pc_signal <= pc_signal;                     
    end

else if(state == 3'b100) 
    begin  
    read_i <= 0; 
    ALUon <= 0; 
    read_d <= read_d;
    if(I[31:25] == 7'b0000111) mux_signal <= 1'b1;            // load
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
