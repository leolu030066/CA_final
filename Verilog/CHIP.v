// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: any combinational/sequential circuit

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
            
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module Imm_Gen(Instruction, Immediate);
    // part(1) in architecture image

    // Generate Corresponding Immediate from raw instructions (No immediate instruction -> 32'b0)
    input [31:0] Instruction ;  //mem_rdata_I (32-bit instruction)
    output [31:0] Immediate ; //ex: used for computing next instruction address or ALU operation(32-bit in Jupiter Simulator)

    reg signed [31:0] Immediate ;
    always @(Instruction)
    begin
        if(Instruction[6:0] == 7'b0010111) begin
            //AUIPC instructions
            Immediate[11:0] = 12'b0 ;
            Immediate[31:12] = Instruction[31:12] ;
        end
        else if(Instruction[6:0] == 7'b1101111) begin
            //JAL instructions
            Immediate[0] = 1'b0 ;
            Immediate[19:12] = Instruction[19:12] ;
            Immediate[11] = Instruction[20] ;
            Immediate[10:1] = Instruction[30:21] ;
            Immediate[20] = Instruction[31] ;
            //signed extension
            if(Immediate[20] == 1'b0) begin
                Immediate[31:21] = 11'b0;
            end
            else begin
                Immediate[31:21] = 11'b1;
            end
        end
        else if(Instruction[6:0] == 7'b1100111 && Instruction[14:12] == 3'b000) begin
            //JALR instructions
            Immediate[11:0] = Instruction[31:20] ;
            //signed extension
            if(Immediate[11] == 1'b0) begin
                Immediate[31:12] = 20'b0;
            end
            else begin
                Immediate[31:12] = 20'b1;
            end
        end
        else if(Instruction[6:0] == 7'b1100011 && Instruction[14:12] == 3'b000) begin
            //BEQ instructions
            Immediate[0] = 0 ;
            Immediate[11] = Instruction[7] ;
            Immediate[4:1] = Instruction[11:8] ;
            Immediate[10:5] = Instruction[30:25] ;
            Immediate[12] = Instruction[31] ;
            //signed extension
            if(Immediate[12] == 1'b0) begin
                Immediate[31:13] = 19'b0;
            end
            else begin
                Immediate[31:13] = 19'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0000011 && Instruction[14:12] == 3'b010) begin
            //LW instructions
            Immediate[11:0] = Instruction[31:20] ;
            //signed extension
            if(Immediate[11] == 1'b0) begin
                Immediate[31:12] = 20'b0;
            end
            else begin
                Immediate[31:12] = 20'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0100011 && Instruction[14:12] == 3'b010) begin
            //SW instructions
            Immediate[4:0] = Instruction[11:7] ;
            Immediate[11:5] = Instruction[31:25] ;
            //signed extension
            if(Immediate[11] == 1'b0) begin
                Immediate[31:12] = 20'b0;
            end
            else begin
                Immediate[31:12] = 20'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0010011 && Instruction[14:12] == 3'b000) begin
            //ADDI instructions
            Immediate[11:0] = Instruction[31:20] ;
            //signed extension
            if(Immediate[11] == 1'b0) begin
                Immediate[31:12] = 20'b0;
            end
            else begin
                Immediate[31:12] = 20'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0010011 && Instruction[14:12] == 3'b010) begin
            //SLTI instructions
            Immediate[11:0] = Instruction[31:20] ;
            //signed extension
            if(Immediate[11] == 1'b0) begin
                Immediate[31:12] = 20'b0;
            end
            else begin
                Immediate[31:12] = 20'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0110011) begin
            //ADD and SUB and XOR and MUL instructions
            Immediate[31:0] = 32'b0 ;
        end
    
        //Todo : Other instruction for hw1(bonus)

        else if(Instruction[6:0] == 7'b1100011 && Instruction[14:12] == 3'b101) begin
            //BGE instructions
            Immediate[0] = 0 ;
            Immediate[11] = Instruction[7] ;
            Immediate[4:1] = Instruction[11:8] ;
            Immediate[10:5] = Instruction[30:25] ;
            Immediate[12] = Instruction[31] ;
            //signed extension
            if(Immediate[12] == 1'b0) begin
                Immediate[31:13] = 19'b0;
            end
            else begin
                Immediate[31:13] = 19'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0010011 && Instruction[14:12] == 3'b101) begin
            //SRAI instructions
            Immediate[4:0] = Instruction[24:20] ;
            //signed extension
            if(Immediate[4] == 1'b0) begin
                Immediate[31:5] = 27'b0;
            end
            else begin
                Immediate[31:5] = 27'b1;
            end
        end
        else if(Instruction[6:0] == 7'b0010011 && Instruction[14:12] == 3'b001) begin
            //SLLI instructions
            Immediate[4:0] = Instruction[24:20] ;
            //signed extension
            if(Immediate[4] == 1'b0) begin
                Immediate[31:5] = 27'b0;
            end
            else begin
                Immediate[31:5] = 27'b1;
            end
        end
        else begin
            Immediate[31:0] = 32'b0 ;
        end
    end
endmodule

module MUX_32_2(s0_data,s1_data,sel,output_data);
    // part(4) in architecture image

    input [31:0] s0_data,s1_data ;
    input sel ;
    output [31:0] output_data ;

    reg signed [31:0] output_data ;

    always @(s0_data or s1_data or sel) 
    begin
        if(sel) output_data = s1_data ;
        else output_data = s0_data ;
    end
endmodule

module ADDER_32(s0_data,s1_data,output_data) ;
    // part(5) in architecture image

    //do not consider overflow
    input [31:0] s0_data,s1_data ;
    output [31:0] output_data ;

    reg signed [31:0] output_data ;

    always @(s0_data or s1_data)
    begin
      output_data = s0_data+s1_data ;
    end

endmodule

module AND_1(s0,s1,output_value);
    // part(6) in architecture image
    input s0,s1 ;
    output output_value ;

    reg output_data ;
    always @(s0 or s1) 
    begin
        output_value = s0 && s1 ;
    end
endmodule

//XOR_1 TBD
module XOR_1(s0,s1,output_value);
    // part(6) in architecture image
    input s0,s1 ;
    output output_value ;

    reg output_data ;
    always @(s0 or s1) 
    begin
        output_value = s0 ^ s1 ;
    end
endmodule

module Control(Opcode, Branch_ctrl, MemRead_ctrl, MemtoReg_ctrl, ALUOP, MemWrite_ctrl, ALUSrc_ctrl, RegWrite_ctrl, JAL_ctrl, JALR_ctrl, AIUPC_ctrl);
	input Opcode;
	output Branch_ctrl, MemRead_ctrl, MemtoReg_ctrl, ALUOP, MemWrite_ctrl, ALUSrc_ctrl, RegWrite_ctrl, JAL_ctrl, JALR_ctrl, AIUPC_ctrl;
	
	always@(*) begin
		case(Opcode)
			7'b0110011: begin
				//R-type
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 2;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 0;
				RegWrite_ctrl = 1;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
			7'b0010011: begin
				//I-type immediate
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 3;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 1;
				RegWrite_ctrl = 1;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
			7'b0000011: begin
				//I-type load
				Branch_ctrl = 0;
				MemRead_ctrl = 1;
				MemtoReg_ctrl = 1;
				ALUOP = 0;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 1;
				RegWrite_ctrl = 1;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
			7'b0100011: begin
				//S-type
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 0;
				MemWrite_ctrl = 1;
				ALUSrc_ctrl = 1;
				RegWrite_ctrl = 0;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
			7'b1100011: begin
				//B-type
				Branch_ctrl = 1;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 1;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 0;
				RegWrite_ctrl = 0;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
			7'b1101111: begin
				//jal
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 0;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 1;
				RegWrite_ctrl = 1;
				JAL_ctrl = 1;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
			7'b1100111: begin
				//jalr
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 0;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 1;
				RegWrite_ctrl = 1;
				JAL_ctrl = 1;
				JALR_ctrl = 1;
				AIUPC_ctrl = 0;
			end
			7'b0010111: begin
				//auipc
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 3;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 1;
				RegWrite_ctrl = 1;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 1;
			end
			default: begin
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 0;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 0;
				RegWrite_ctrl = 0;
				JAL_ctrl = 0;
				JALR_ctrl = 0;
				AIUPC_ctrl = 0;
			end
		endcase
	end
endmodule

module ALUControl(ALUOP, Instruction, ALU_ctrl);
	//ALU 0: add, 1:sub, 2:mul, 3: shift_left, 4:shift_right
    input ALUOP;
	output Instruction, ALU_ctrl;
	always@(*) begin
		case(ALUOP)
			0: begin
				//I-type load, S-type, jal, jalr
				ALU_ctrl = 0;
			end
			1: begin
				//B-type
				ALU_ctrl = 1;
			end
			2: begin
				//R-type
				case(Instruction[14:12])
					3'b000: begin
						if(Instruction[30] == 1) ALU_ctrl = 1;//sub instruction
						else begin
							if(Instruction[25] == 1) ALU_ctrl = 2;//mul instruction
							else ALU_ctrl = 0;//add instruction
						end
					end
					default: ALU_ctrl = 0;
				endcase
			end
			3: begin
				//I-type immediate, auipc
				case(Instruction[14:12])
					3'b000: ALU_ctrl = 0;//addi instruxtion
					3'b001: ALU_ctrl = 3;//slli instruxtion
					3'b101: ALU_ctrl = 4;//srli instruxtion
					default: ALU_ctrl = 0;
				endcase
			end
			default: ALU_ctrl = 0;
		endcase
	end

endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2

endmodule