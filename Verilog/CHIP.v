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
    // Generate Corresponding Immediate from raw instructions
    input [31:0] Instruction ;  //mem_rdata_I (32-bit instruction)
    output [31:0] Immediate ; //ex: used for computing next instruction address or ALU operation(32-bit in Jupiter Simulator)

    reg signed [31:0] Immediate ;
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
    //Todo : Other instruction for hw1(bonus)
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


    


endmodule



module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2

endmodule