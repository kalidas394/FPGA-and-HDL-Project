`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:40:04 08/03/2015 
// Design Name: 
// Module Name:    risccpu 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module risccpu(
output[15:0] ir_out,
input clk,
input reset
);


	reg rst,pc_ld,pc_inc,pc_clr,ir_ld,data_addr_sel,w_wr,rd_P,rd_Q,rst16x16,rd,wr;
	reg [1:0] rf_sel;
	reg [2:0] alu_sel;
	reg [3:0] wr_addr,rd_addr_P,rd_addr_Q;
	
	wire [7:0] data_addr, rf_w_data;
	
	assign data_addr = ir_out[7:0];
	assign rf_w_data = ir_out[7:0];

	PROCESSOR uP (.rst(rst), .clk(clk), .pc_ld(pc_ld), .pc_inc(pc_inc), .pc_clr(pc_clr), .ir_ld(ir_ld), .ir_out(ir_out), .data_addr_sel(data_addr_sel), 
		.data_addr(data_addr), .rd(rd), .wr(wr), .rf_sel(rf_sel), .rf_w_data(rf_w_data), .wr_addr(wr_addr), .rd_addr_P(rd_addr_P), .rd_addr_Q(rd_addr_Q), 
		.w_wr(w_wr), .rd_P(rd_P), .rd_Q(rd_Q), .rst16x16(rst16x16), .alu_sel(alu_sel)
	);

		
always @(posedge clk, posedge reset)
begin
	if (reset)
		begin
			pc_ld	 = 0;
			pc_clr = 1;
			pc_inc = 0;
			ir_ld = 0;
			data_addr_sel = 0;
			rd = 1;
			wr = 0;
			rf_sel = 1;
			rst16x16 = 1;
			alu_sel = 0;
			
				@(posedge clk) begin
					pc_clr = 0;
					pc_inc = 1;
					ir_ld = 1;
					rst16x16 = 0;
				end
		end
		
	else
		case (ir_out[15:8])
		0: begin
				pc_inc = 0;
			end
		
		1:	begin
			pc_inc = 0;
			ir_ld = 0;
			data_addr_sel = 1;
			rf_sel = 1;
			alu_sel = 0;
			
			wr_addr	 = ir_out[7:4];
			rd_addr_P = ir_out[7:4];
			rd_addr_Q = ir_out[3:0];
			w_wr = 1; 
			rd_P = 0;
			rd_Q = 0;

						@(posedge clk)begin
							@(posedge clk)begin
								data_addr_sel = 0;
								rf_sel = 0;
								pc_inc = 1;
								@(posedge clk)
								begin
									w_wr = 0;
									ir_ld = 1;
									rd_P = 1;
									rd_Q = 1;
								end
							end	
						end
			end
	
		
	   2:	begin
		//COPY MEMORY, PASTE TO REGISTER P;
			pc_inc = 0;
			ir_ld = 0;
			data_addr_sel = 1;
			rf_sel = 1;
			alu_sel = 1;//DONT CARE
			
			wr_addr	 = ir_out[11:8];
			rd_addr_P = ir_out[7:4];
			rd_addr_Q = ir_out[3:0];
			w_wr = 1; 
			rd_P = 0;
			rd_Q = 0;

						@(posedge clk)begin
							@(posedge clk)begin
								data_addr_sel = 0;
								rf_sel = 0;
								pc_inc = 1;
								@(posedge clk)
								begin
									w_wr = 0;
									ir_ld = 1;
									rd_P = 1;
									rd_Q = 1;
								end
							end
						end
			end
      /*   //////////////********* for showing in ram*****************//////////
        /* 3:	begin
		//COPY MEMORY, PASTE TO REGISTER P;
			pc_inc = 0;
			ir_ld = 0;
			data_addr_sel = 1;
			rf_sel = 1;
			alu_sel = 1;//DONT CARE
			
			wr_addr	 = ir_out[11:8];
			rd_addr_P = ir_out[7:4];
			rd_addr_Q = ir_out[3:0];
			w_wr = 1; 
			rd_P = 0;
			rd_Q = 0;

						@(posedge clk)begin
							@(posedge clk)begin
								data_addr_sel = 0;
								rf_sel = 0;
								pc_inc = 1;
								@(posedge clk)
								begin
									w_wr = 0;
									//ir_ld = 0;
									rd_P = 1;
									rd_Q = 0;
								end
							end
						end
			end	
	//////////////////////////////////////////*****ends here*****//////	   
		
		default:	;
		endcase

end


endmodule


module PROCESSOR(
input rst,clk,						

input pc_ld, pc_inc,pc_clr,	
input ir_ld, 						
output [15:0] ir_out,
input data_addr_sel,				
input [7:0] data_addr,
input rd, wr,						
input [1:0] rf_sel, 				
input [7:0] rf_w_data,	
input [3:0] wr_addr, rd_addr_P, rd_addr_Q,  		
input w_wr, rd_P, rd_Q, rst16x16,
input [2:0] alu_sel				
);

wire[7:0] pc_addr;		//PC

wire [7:0] mux8_out;		//MUX2X1

wire [15:0] mem_out;		//MEM 256X16

wire [15:0] mux16_out;	//MUX3X1

wire [15:0] data_out_P, data_out_Q;	//MEM 16X16

wire [15:0] alu_out;		//ALU


programcounter U0 (.data_out(pc_addr), .data_in(ir_out[15:8]), .pc_ld(pc_ld), .pc_inc(pc_inc), .clk(clk), .rst(pc_clr));
instructionregister U1 (.data_out(ir_out), .data_in(mem_out), .ir_ld(ir_ld), .clk(clk), .rst(rst));
mux2_1 U2 (.sel(data_addr_sel), .pc_addr_in(pc_addr), .data_addr_in(data_addr), .mux8_out(mux8_out));
ram U3(.data_out(mem_out), .data_in(data_out_P), .address(mux8_out), .rd(rd), .wr(wr), .clk(clk));
mux3_1 U4(.sel(rf_sel), .alu_in(alu_out), .mem_in(mem_out), .cntrl_in(rf_w_data), .mux16_out(mux16_out));
cache U5(.data_out_P(data_out_P), .data_out_Q(data_out_Q), .data_in(mux16_out), .wr_addr(wr_addr),
	.rd_addr_P(rd_addr_P), .rd_addr_Q(rd_addr_Q), .rf_sel(rf_sel),.wr(w_wr), .rd_P(rd_P), .rd_Q(rd_Q), .clk(clk), .rst(rst16x16));
alu U6 (.sel(alu_sel), .data0(data_out_Q), .data1(data_out_P), .alu_out(alu_out));
endmodule

module instructionregister(
output reg [15:0] data_out,
input [15:0] data_in,
input ir_ld, clk, rst
);

	always @(posedge clk, posedge rst)
	if (rst)
		data_out = 0;
	
	else if (ir_ld)
		data_out = data_in;
		
endmodule


module programcounter(
output reg [7:0] data_out,
input [7:0] data_in,
input pc_ld, pc_inc, clk, rst
);

	always @(posedge clk, posedge rst)
	begin
		if (rst)
			data_out = 0;
		
		else if (pc_ld)
			data_out = data_in;
			
			else if(pc_inc)
			data_out = data_out+1;
			
		
	end
endmodule


module cache(
output [15:0] data_out_P, data_out_Q,
input [15:0] data_in,
input [3:0] wr_addr, rd_addr_P, rd_addr_Q,
input [1:0] rf_sel,
input wr, rd_P, rd_Q, clk, rst
);

reg [4:0] r;
reg [15:0] mem_P [15:0], mem_Q [15:0];

wire [15:0] read_P, read_Q;

assign data_out_P = mem_P[rd_addr_P];
assign data_out_Q = mem_Q[rd_addr_Q];

assign read_P = (rd_P ? mem_P[rd_addr_P]:16'bz);
assign read_Q = (rd_Q ? mem_Q[rd_addr_Q]:16'bz);

	always @(posedge clk, posedge rst)
	begin
		if (rst)
		begin
			for(r=0;r<15;r=r+1)
			begin
					mem_P[r] = 16'b0011001100110011;
					mem_Q[r] = 16'b1010101010101010;
			end
		end
	
		else 
		begin
			if (wr  & (|rf_sel))
					mem_P[wr_addr] = data_in;
				
			else if(wr&(~|rf_sel))
					mem_Q[rd_addr_Q] = data_in;
		end
	end	
endmodule


module ram(
output reg [15:0] data_out,
input [15:0] data_in,
input [7:0] address,
input rd, wr, clk);

reg [15:0] memory [255:0];

initial begin
					//opcode[11:8],r_P_addr[7:4], r_Q_addr[3:0] 
					 
	memory[0]  = 16'b00000000_0000_0000;
	memory[1]  = 16'b00000001_0000_0000;
	memory[2]  = 31;
	memory[3]  = 16'b00000010_0001_0001;
	memory[4]  = 32;
	
	
	
	

	memory[31]  = 22500; 
	memory[32]  = 16'b0011001100110011;	
	memory[33]  = 45;		
	memory[34]  = 14;		
	memory[54]  = 16'b0101_0101_0101_0101; 
	
	
end


	always @(posedge clk)
	begin
		if (rd)
			data_out <= memory[address];
		else if (wr)
			memory[address] <= data_in;
	end
endmodule


module mux2_1(
input sel,
input [7:0] pc_addr_in,data_addr_in,
output [7:0] mux8_out
);

	assign mux8_out = (sel==0? pc_addr_in:
							 sel==1? data_addr_in:0);
endmodule



module mux3_1(
input [1:0] sel,
input [15:0] alu_in, mem_in, cntrl_in,
output [15:0] mux16_out
);

	assign mux16_out = (sel==0? alu_in:
							  sel==1? mem_in:
							  sel==2? cntrl_in: 0);
endmodule


module alu
#(parameter ADD = 0, SUB = 1, DNC = 2, DNCop= 3)
(
	input [2:0] sel,
	input [15:0] data0, data1,
	output reg[15:0] alu_out
);

	always @(sel, data0, data1)
	case(sel)
		ADD:		alu_out = data0 + data1;
		SUB:		alu_out = data0 - data1;
		DNC:     alu_out <= data0;
		DNCop:   alu_out = 0;
		
		
		default: alu_out = 0;
	endcase		
endmodule


module CLOCK_GEN(output reg clock);

initial begin
	clock =0;
	
	forever #5 clock = ~clock;
end
endmodule
