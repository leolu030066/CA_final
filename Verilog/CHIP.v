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
    reg   [31:0] PC_nxt      ;              //
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
	
	
	//Control related
	wire branch_ctrl, memread_ctrl, memtoreg_ctrl, alusrc_ctrl;
	reg memwrite_ctrl;
	wire [1:0] aluop;
	wire address_control;
	//Imm_Gen related
	reg signed [31:0] immediate;
	//PreALU related
	reg [31:0] prealuout;
	//ALU related
	reg [31:0] aluout;
	//ALUControl related
	wire alu_ctrl;

    wire mul_aluout;

    wire mul_ready;

    wire final_aluout;
	//WB related
	// wire wbrd;
	//NormalPC related (PC+4)
	reg [31:0] normalpc;
	//Branch/JAL/AUIPC target address (PC+immediate)
	reg [31:0] pc_imm;
    //JALR target address (x1+immediate)
    reg [31:0] x1_imm;

	//SelPC related
    wire dobranch ;
	wire _mul;
	reg [1:0] selpc;

    assign mem_addr_I = PC;
	assign rs1 = mem_rdata_I[19:15];
	assign rs2 = mem_rdata_I[24:20];
	assign rd = mem_rdata_I[11:7];
	
	//Control
	Control Control(.Opcode(mem_rdata_I[6:0]), .Branch_ctrl(branch_ctrl), .MemRead_ctrl(memread_ctrl), .MemtoReg_ctrl(memtoreg_ctrl), .ALUOP(aluop), .MemWrite_ctrl(memwrite_ctrl), .ALUSrc_ctrl(alusrc_ctrl), .RegWrite_ctrl(regWrite), .selpc(selpc));
	// OR_1 UseData(.s0(memread_ctrl), .s1(memwrite_ctrl), .output_value(getdata));
	assign mem_wen_D = memwrite_ctrl;

	//ID
	Imm_Gen Imm_Gen(.Instruction(mem_rdata_I), .Immediate(immediate));
	
	
	ADDER_32 NormalPC(.s0_data(32'd4), .s1_data(PC), .output_data(normalpc));
	ADDER_32 ImmPC(.s0_data(PC), .s1_data(immediate), .output_data(pc_imm));
    ADDER_32 Immrs1(.s0_data(rs1_data),.s1_data(immediate),.output_data(x1_imm));

	//EX
	MUX_32_2 PreALU(.s0_data(rs2), .s1_data(immediate), .sel(alusrc_ctrl), .output_data(prealuout));
	
	ALUControl ALUControl(.ALUOP(aluop), .Instruction(PC), .ALU_ctrl(alu_ctrl),.mul(_mul));

    BasicALU EXE(.input_1(rs1),.input_2(prealuout),.mode(alu_ctrl),.out(aluout),.out_zero(aluzero));
    //Todo MUX
    MUL MUL(
        .clk(clk),
        .rst_n(rst_n),
        .valid(_mul),
        .ready(mul_ready),
        .mode(alu_ctrl),
        .in_A(rs1),
        .in_B(prealuout),
        .out(mul_aluout)
        );
    MUX_32_2 deside_aluout(.s0_data(mul_aluout),.s1_data(aluout),.sel(_mul),.output_data(final_aluout));

	AND_1 Branchdetect(.s0(branch_ctrl), .s1(aluzero), .output_value(dobranch));
	// MUX_32_2 SelPC(.s0_data(normalpc), .s1_data(branchpc), .sel(dobranch), .output_data(PC_nxt));

    // always @(normalpc or pc_imm or x1_imm or pc or selpc)begin
    //     if (_mul) begin
    //         if(ready)begin
    //             MUX_32_4 SelPCM(.s0_data(normalpc),.s1_data(pc_imm),.s2_data(x1_imm),.s3_data(PC),.sel(2'd0), .output_data(PC_nxt));
    //         end
    //         else begin
    //             MUX_32_4 SelPCMNY(.s0_data(normalpc),.s1_data(pc_imm),.s2_data(x1_imm),.s3_data(PC),.sel(2'd3), .output_data(PC_nxt));
    //         end
    //     end
    //     else if (sel == 2'b01)begin
    //         // beq/bge
    //         if(dobranch)begin
    //             MUX_32_4 SelPCB(.s0_data(normalpc),.s1_data(pc_imm),.s2_data(x1_imm),.s3_data(PC),.sel(2'd1), .output_data(PC_nxt));
    //         end
    //         else begin
    //             MUX_32_4 SelPCNB(.s0_data(normalpc),.s1_data(pc_imm),.s2_data(x1_imm),.s3_data(PC),.sel(2'd0), .output_data(PC_nxt));
    //         end
    //     end 
    //     else begin
    //         MUX_32_4 SelPCE(.s0_data(normalpc),.s1_data(pc_imm),.s2_data(x1_imm),.s3_data(PC),.sel(selpc), .output_data(PC_nxt));
    //     end
    // end
    always @(normalpc or pc_imm or x1_imm or PC or selpc)begin
        if (_mul == 1'b1) begin
            if(mul_ready == 1'b1)begin
                PC_nxt = normalpc ;
            end
            else begin
                PC_nxt = PC ;
            end
        end
        else begin
            if (selpc == 2'd0)begin
                PC_nxt = normalpc ;
            end
            // beq/bge
            else if(selpc == 2'd1)begin
                if (dobranch == 1'd1) begin
                    PC_nxt = pc_imm ;
                end
                else begin
                    PC_nxt = normalpc ;
                end
            end
            else if (selpc == 2'd2)begin
                PC_nxt = x1_imm ;
            end
            else begin
                PC_nxt = PC ;
            end
        end 
    end
        

    

	//ME
	MUX_32_2 PostALU(.s0_data(0), .s1_data(rs2_data), .sel(memwrite_ctrl), .output_data(mem_wdata_D));
	MUX_32_2 DataAddr(.s0_data(0), .s1_data(rd_data+immediate), .sel(memwrite_ctrl), .output_data(mem_addr_D));
	//WB
	MUX_32_2 WB(.s0_data(final_aluout), .s1_data(mem_rdata_D), .sel(memtoreg_ctrl), .output_data(rd_data));
	//TODO: jal, wbrd
	
	
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

module MUX_32_2(output_data,s0_data,s1_data,sel);
    // part(4) in architecture image

    input [31:0] s0_data,s1_data ;
    input sel ;
    output [31:0] output_data ;

    reg [31:0] output_data ;

    always @(*) 
    begin
        if(sel==1'd1) output_data = s1_data ;
        else output_data = s0_data ;
    end
endmodule

module MUX_32_4(output_data,s0_data,s1_data,s2_data,s3_data,sel) ;
    input [31:0] s0_data,s1_data,s2_data ,s3_data;
    input [1:0]sel ;
    output [31:0] output_data ;
    reg [31:0] output_data ;

    always @(*) 
    begin
        if(sel == 2'd0) begin
            output_data = s0_data ;
        end
        else if(sel == 2'd1) begin
            output_data = s1_data ;
        end
        else if(sel == 2'd2) begin
            output_data = s2_data ;
        end
        else begin
            output_data = s3_data ;
        end
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

    reg output_value ;
    always @(s0 or s1) 
    begin
        output_value = (s0 && s1) ;
    end
endmodule


//OR_1 TBD
module OR_1(s0,s1,output_value);


    // part(6) in architecture image
    input s0,s1 ;
    output output_value ;

    reg output_data ;
    always @(s0 or s1) 
    begin
        output_value = (s0 || s1) ;
    end
endmodule

module Control(Opcode, Branch_ctrl, MemRead_ctrl, MemtoReg_ctrl, ALUOP, MemWrite_ctrl, ALUSrc_ctrl, RegWrite_ctrl, selpc);
	input [6:0] Opcode;
	output Branch_ctrl, MemRead_ctrl, MemtoReg_ctrl, MemWrite_ctrl, ALUSrc_ctrl, RegWrite_ctrl;
	output [1:0] ALUOP;
	output [1:0] selpc;
	reg Branch_ctrl, MemRead_ctrl, MemtoReg_ctrl, MemWrite_ctrl, ALUSrc_ctrl, RegWrite_ctrl;
	reg [1:0] ALUOP;
	reg [1:0] selpc;
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
				selpc = 0;
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
				selpc = 0;
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
				selpc = 0;
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
				selpc = 0;
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
				selpc = 1;
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
				selpc = 1;
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
				selpc = 2;
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
				selpc = 1;
			end
			default: begin
				Branch_ctrl = 0;
				MemRead_ctrl = 0;
				MemtoReg_ctrl = 0;
				ALUOP = 0;
				MemWrite_ctrl = 0;
				ALUSrc_ctrl = 0;
				RegWrite_ctrl = 0;
				selpc = 0;
			end
		endcase
	end
endmodule

module ALUControl(ALUOP, Instruction, ALU_ctrl,mul);
    //ALU 0: add, 1:sub, 2:mul, 3: shift_left, 4:shift_right, 5:bge
    input [31:0] Instruction;
	input [1:0] ALUOP;
    output [1:0] ALU_ctrl;
    output mul;
    reg [1:0] ALU_ctrl;
    always@(*) begin
        case(ALUOP)
            0: begin
                //I-type load, S-type, jal, jalr
                ALU_ctrl = 0;
            end
            1: begin
                //B-type
				case(Instruction[14:12])
					3'b000: begin
						ALU_ctrl = 1;
					end
					3'b101: begin
						ALU_ctrl = 5;
					end
					default: ALU_ctrl = 1;
				endcase
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
            if(ALU_ctrl == 2) mul = 1;
            else mul = 0;
            default: ALU_ctrl = 0;
        endcase
    end

endmodule

module BasicALU (
    input_1,
    input_2,
    mode,
    out_zero,
    out
);
    input [31:0] input_1;
    input [31:0] input_2;
    input [2:0] mode;
    output out_zero;
    output [31:0] out;

    reg [31:0] regist;

    parameter ADD = 3'd0;
    parameter SUB = 3'd1;
    parameter MUL  = 3'd2;
    parameter SLLI  = 3'd3;
    parameter SLRI = 3'd4;
    parameter BGE = 3'd5;

    assign out = regist;
    assign out_zero = (regist == 32'b0)? 1'b1:1'b0;
    assign bge_zero = (regist >= 32'b0)? 1'b1:1'b0;

    always @(*) begin
        case(mode)
            ADD: regist = input_1 + input_2;
            SUB: regist = input_1 - input_2;
            SLLI: regist = input_1 << input_2;
            SLRI: regist = input_1 >> input_2;
            BGE: regist = (input_1 > input_2)? 1'b1:1'b0;
            default: regist = {32{1'b0}};
        endcase
    end

endmodule
module MUL(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out,
    out_zero
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [2:0]  mode; // 0: add, 1:sub, 2:mul, 3: shift_left, 4:shift_right
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;
    output        out_zero;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter ADD = 3'd1;
    parameter SUB = 3'd2;
    parameter MUL  = 3'd3;
    parameter SLLI  = 3'd4;
    parameter SLRI = 3'd5;
	parameter OUT = 3'd6;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if(valid)begin
                    case(mode)
                        0 : state_nxt = ADD;
                        1 : state_nxt = SUB;
                        1 : state_nxt = MUL;
                        2 : state_nxt = SLLI;
                        3 : state_nxt = SLRI;
                        default : state_nxt = IDLE;
                    endcase
                end
                else state_nxt = IDLE;
            end
            ADD :
                state_nxt = OUT;
            
            SUB:
                state_nxt = OUT;
            MUL :if (counter == 31) begin
                state_nxt = OUT;
            end
            SLLI :
                state_nxt = OUT;
            SLRI : 
                state_nxt = OUT;
            OUT : state_nxt = IDLE;
            default :
                state_nxt = state;
        endcase
    end
    assign ready = (state == OUT)? 1'b1 : 1'b0;
    assign out = (state == OUT)? shreg : 0; 
    // Todo 2: Counter
    always @(*) begin
        counter_nxt = 0;
        if (state == MUL) counter_nxt = counter + 1;
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
     always @(*) begin
        case(state)
            MUL : begin
                if(shreg[0] == 1'b1)
                    alu_out = shreg[63:32] + alu_in;                    
                else
                    alu_out = shreg[63:32];
            end           
            ADD : begin
                alu_out = shreg + alu_in;
            end
            SUB : begin
                alu_out = shreg - alu_in;
            end
            SLLI :begin
                alu_out = shreg << alu_in;
            end
            SLRI :begin
                alu_out = shreg >> alu_in;
            end
            default : alu_out = 0;
        endcase
    end    
    // Todo 4: Shift register
    always @(*) begin
        shreg_nxt = shreg;
        case(state)
            IDLE: if (valid == 1) shreg_nxt = {32'b0,  in_A};
            MUL: shreg_nxt = {alu_out, shreg[31:1]};
            ADD: shreg_nxt = {32'b0, alu_out};
            SUB: shreg_nxt = {32'b0, alu_out};
            SLLI: shreg_nxt = {32'b0, alu_out};
            SLRI: shreg_nxt = {32'b0, alu_out};
        endcase
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter     <= 5'b0;
            shreg       <= 64'b0;
            alu_in      <= 32'b0;
        end
        else begin
            state <= state_nxt;
            shreg <= shreg_nxt;
            counter <= counter_nxt;
            alu_in <= alu_in_nxt;
        end
    end

endmodule
