`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   16:46:44 08/03/2015
// Design Name:   risccpu
// Module Name:   C:/Xilinx/risc16bit/risccpu_tb.v
// Project Name:  risc16bit
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: risccpu
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module risccpu_tb;

	
	reg clk, reset;
	wire [7:0] pc_out, mux8_out;
	wire [15:0] mem_out, ir_out, mux16, read_P, read_Q; //mem_61;

	assign pc_out 	 = U1.uP.U0.data_out;
	assign mux8_out = U1.uP.U2.mux8_out;
	assign mem_out	 = U1.uP.U3.data_out;
	assign ir_out 	 = U1.uP.U1.data_out;
	assign mux16 	 = U1.uP.U4.mux16_out;
	assign read_P 	 = U1.uP.U5.read_P;
	assign read_Q 	 = U1.uP.U5.read_Q;
	//assign mem_39 	 = U1.uP.U3.memory[39];
	
	risccpu U1 (.ir_out(ir_out), .clk(clk),.reset(reset));

	initial begin
	$display ("time	pc_clr	pc_ld	pc_inc	ir_ld	data_addr_sel	rd	wr	rf_sel	w_wr	rd_P	rd_Q	alu_sel	pc_out	mux8_out	mem_out	ir_out	mux16	read_P	read_Q	mem[61]");
	$monitor ("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%b\t%b\t%b\t%b\t%b",
	$time, 	U1.uP.U0.rst,
				U1.uP.U0.pc_ld,
				U1.uP.U0.pc_inc,
				U1.uP.U1.ir_ld,
				U1.uP.U2.sel,
				U1.uP.U3.rd,
				U1.uP.U3.wr,
				U1.uP.U4.sel,
				U1.uP.U5.wr,
				U1.uP.U5.rd_P,
				U1.uP.U5.rd_Q,
				U1.uP.U6.sel,
				U1.uP.U0.data_out,
				U1.uP.U2.mux8_out,
				U1.uP.U3.data_out,
				U1.uP.U1.data_out,
				U1.uP.U4.mux16_out,
				U1.uP.U5.read_P,
				U1.uP.U5.read_Q);
	
			 clk = 0; reset = 1;
		#5  reset = 0;
		 #1000 $finish;
	end 
      
	always #5 clk = ~clk;
endmodule
