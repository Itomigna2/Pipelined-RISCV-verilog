	module riscv(clk);		
	input clk;    

	wire [31:0] qinstruction;
	wire ALUSrc,MemtoReg,RegWriteEnable,MemRead,MemWrite,branch,ALUOp1,ALUOp0,Zero;	
	reg [31:0] toReg;
	wire [31:0] IMMGEN;
	wire [31:0] ALUresult;
	wire [31:0] DMdataOutput;			
	wire [4:0] ALUcontrol4bit;
	wire [1:0] LUIsignal;
	wire [1:0] JALsignal;
	reg [31:0] PC;
	reg [31:0] toALU;
	reg [31:0] Register [0:31]; 
    reg [31:0] DMdataOutputExtended;		
	
	////IFID PIPELINE REGISTER////
	reg [31:0] IFID_PC;
	reg [31:0] IFID_QINSTRUCTION;	
	always @(posedge clk) begin
		if((EXMEM_ZERO & EXMEM_BRANCH) || EXMEM_JALSIGNAL) begin
		IFID_PC <= 0;
		IFID_QINSTRUCTION<=0;
		end 
		
		else if(IFIDWRITEOFF == 0) begin
		IFID_PC <= PC;
		IFID_QINSTRUCTION<=qinstruction;
		end
	end	
		
	////IDEX PIPELINE REGISTER////
	reg [31:0] IDEX_PC;
	reg [31:0] IDEX_READDATA1;
	reg [31:0] IDEX_READDATA2;
	reg [31:0] IDEX_IMMGEN;
	reg [6:0] IDEX_FUNCT7;
	reg [2:0] IDEX_FUNCT3;	
	reg [4:0] IDEX_RD;
	reg [4:0] IDEX_RS1;
	reg [4:0] IDEX_RS2;
	reg IDEX_REGWRITE;
	reg	IDEX_MEMTOREG;
	reg	IDEX_BRANCH;
	reg	IDEX_MEMREAD;
	reg	IDEX_MEMWRITE;
	reg [1:0]IDEX_ALUOP;
	reg	IDEX_ALUSRC;
    reg [1:0]IDEX_LUISIGNAL;
	reg [1:0]IDEX_JALSIGNAL;		
	always @(posedge clk) begin
		if((EXMEM_ZERO & EXMEM_BRANCH) || EXMEM_JALSIGNAL) begin
		IDEX_PC <= 0;
		IDEX_READDATA1<=0;
		IDEX_READDATA2<=0;
		IDEX_IMMGEN<=0;
		IDEX_FUNCT7 <= 0;
		IDEX_FUNCT3 <= 0;
		IDEX_RD <= 0;
		IDEX_RS1 <= 0;
		IDEX_RS2 <= 0;
		IDEX_REGWRITE <= 0;
		IDEX_MEMTOREG <= 0;
		IDEX_BRANCH <= 0;
		IDEX_MEMREAD <= 0;
		IDEX_MEMWRITE <= 0;
		IDEX_ALUOP <= 2'b00;
		IDEX_ALUSRC <= 0;
		IDEX_LUISIGNAL <=2'b00;
		IDEX_JALSIGNAL <=2'b00;
		end 
		else begin		
		IDEX_PC <= IFID_PC;
		IDEX_READDATA1<= LUIsignal[0] ? 0 : ( LUIsignal[1] ? IFID_PC : Register[IFID_QINSTRUCTION[19:15]]); //LUI signal ,lui,auipc
		IDEX_READDATA2<= Register[IFID_QINSTRUCTION[24:20]];
		IDEX_IMMGEN<=IMMGEN;
		IDEX_FUNCT7 <= IFID_QINSTRUCTION[31:25];
		IDEX_FUNCT3 <= IFID_QINSTRUCTION[14:12];
		IDEX_RD <= IFID_QINSTRUCTION[11:7];
		IDEX_RS1 <= IFID_QINSTRUCTION[19:15];
		IDEX_RS2 <= IFID_QINSTRUCTION[24:20];
		if(TOCONTROLBITMUX==0)begin
		IDEX_REGWRITE <= RegWriteEnable;
		IDEX_MEMTOREG <= MemtoReg;
		IDEX_BRANCH <= branch;
		IDEX_MEMREAD <= MemRead;
		IDEX_MEMWRITE <= MemWrite;
		IDEX_ALUOP <= {ALUOp1,ALUOp0};
		IDEX_ALUSRC <= ALUSrc;
		IDEX_LUISIGNAL <= LUIsignal;
		IDEX_JALSIGNAL <= JALsignal;
		end 
		else begin
		IDEX_REGWRITE <= 0;
		IDEX_MEMTOREG <= 0;
		IDEX_BRANCH <= 0;
		IDEX_MEMREAD <= 0;
		IDEX_MEMWRITE <= 0;
		IDEX_ALUOP <= 2'b00;
		IDEX_ALUSRC <= 0;
		IDEX_LUISIGNAL <=2'b00;
		IDEX_JALSIGNAL <=2'b00;
		end
		end
	end
		
	////EXMEM PIPELINE REGISTER////
	reg [31:0] EXMEM_ADDSUM_OUTPUT;
	reg EXMEM_ZERO;
	reg [31:0] EXMEM_ALURESULT;
	reg [31:0] EXMEM_READDATA2;
	reg [31:0] EXMEM_RD;
	reg   EXMEM_REGWRITE;
	reg	EXMEM_MEMTOREG;
	reg	EXMEM_BRANCH;
	reg	EXMEM_MEMREAD;
	reg	EXMEM_MEMWRITE;	
	reg  [1:0] EXMEM_JALSIGNAL;	
	reg [2:0] EXMEM_FUNCT3;
	reg [1:0] EXMEM_ALUOP;
	always @(posedge clk) begin
		if((EXMEM_ZERO & EXMEM_BRANCH) || EXMEM_JALSIGNAL) begin
		EXMEM_ADDSUM_OUTPUT <= 0;		
		EXMEM_ZERO <=0;
		EXMEM_ALURESULT <= 0;
		EXMEM_READDATA2<=0;
		EXMEM_RD<=0;		
		EXMEM_REGWRITE <=0;
		EXMEM_MEMTOREG <= 0;
		EXMEM_BRANCH <= 0;
		EXMEM_MEMREAD <= 0;
		EXMEM_MEMWRITE <= 0;		
		EXMEM_FUNCT3 <=0;
		EXMEM_ALUOP <=0;		
		EXMEM_JALSIGNAL <=2'b00;		
		end
		else begin	
		EXMEM_ADDSUM_OUTPUT <= (IDEX_JALSIGNAL[1] ? IDEX_READDATA1 : IDEX_PC)+IDEX_IMMGEN;	//AddSum //jal, jalr	
		EXMEM_ZERO <= Zero;
		EXMEM_ALURESULT <= IDEX_JALSIGNAL ? (IDEX_PC+4) : ALUresult;//jal,jalr
		EXMEM_READDATA2<=ALUinput2;
		EXMEM_RD<=IDEX_RD;		
		EXMEM_REGWRITE <=IDEX_REGWRITE;
		EXMEM_MEMTOREG <= IDEX_MEMTOREG;
		EXMEM_BRANCH <= IDEX_BRANCH;
		EXMEM_MEMREAD <= IDEX_MEMREAD;
		EXMEM_MEMWRITE <= IDEX_MEMWRITE;			
		EXMEM_FUNCT3 <=IDEX_FUNCT3;
		EXMEM_ALUOP <=IDEX_ALUOP;		
		EXMEM_JALSIGNAL <= IDEX_JALSIGNAL;				
		end
	end	
	
	////MEMWB PIPELINE REGISTER////
	reg [31:0] MEMWB_DMOUTPUT;
	reg [31:0] MEMWB_ALURESULT;
	reg [31:0] MEMWB_RD;
	reg [31:0] MEMWB_REGWRITE;
	reg [31:0] MEMWB_MEMTOREG;	
	always @(posedge clk) begin
		MEMWB_DMOUTPUT <= DMdataOutputExtended;
		MEMWB_ALURESULT <= EXMEM_ALURESULT;
		MEMWB_RD <= EXMEM_RD;
		
		MEMWB_REGWRITE <=  EXMEM_REGWRITE;
		MEMWB_MEMTOREG <= EXMEM_MEMTOREG;
	end	
	
	//Program Counter   with   PCSrc(branch) MUX   	
	always @(posedge clk) begin
		
		if(PCWRITEOFF == 0) begin		
		if((EXMEM_ZERO & EXMEM_BRANCH) || EXMEM_JALSIGNAL)
				PC <= EXMEM_ADDSUM_OUTPUT;		
		else PC<=PC+4;		
		end
	end	
	initial PC=-4;	
	
	////Instruction Memory
	instRAM ({{2'b0},{PC[31:2]}},	!clk , 0 ,	0,  qinstruction);	
	////Controller	
	controller(IFID_QINSTRUCTION[6:0],ALUSrc,MemtoReg,RegWriteEnable,MemRead,MemWrite,branch,ALUOp1,ALUOp0,Signal8bit,LUIsignal,JALsignal);
	ALU_controller(IDEX_ALUOP, IDEX_FUNCT7, IDEX_FUNCT3,  ALUcontrol4bit,IDEX_LUISIGNAL);
	
	////Register Write  with memtoreg mux //Write Back  
	always @(negedge clk) begin	
    	toReg = (MEMWB_MEMTOREG ? MEMWB_DMOUTPUT : MEMWB_ALURESULT);
		if(MEMWB_REGWRITE==1) Register[MEMWB_RD] =  toReg;
		Register[0]=0;
	end	

	//IMMGEN
	IMMGEN(IFID_QINSTRUCTION, IMMGEN);

	//ALU
	ALU(ALUinput1, toALU, ALUresult, ALUcontrol4bit,Zero);	

	//ALUSrc MUX	
	always @* begin
		toALU <= IDEX_ALUSRC ? IDEX_IMMGEN : ALUinput2;
	end

	//Data Memory	
	RAM ({{2'b0},{EXMEM_ALURESULT[31:2]}}, ByteEnable,	!clk , EXMEM_READDATA2 ,	EXMEM_MEMWRITE,  DMdataOutput);	

	//loaded data decoding
	always @* begin
		case(SignExtensionSelect) 
		3'b000: DMdataOutputExtended <= {{24{DMdataOutput[7]}},{DMdataOutput[7:0]}};
		3'b001: DMdataOutputExtended <= {{16{DMdataOutput[15]}},{DMdataOutput[15:0]}};
		3'b100: DMdataOutputExtended <= {{24{1'b0}},{DMdataOutput[7:0]}};
		3'b101: DMdataOutputExtended <= {{16{1'b0}},{DMdataOutput[15:0]}};
		default : DMdataOutputExtended <= DMdataOutput;
		endcase
	end

	reg [3:0] ByteEnable;
	reg [2:0] SignExtensionSelect;
	reg [4:0] temp;
	always @* begin
	
	temp = {{EXMEM_ALUOP}, {EXMEM_FUNCT3}};
		case (temp)
		12'b00000 : begin SignExtensionSelect=3'b000; ByteEnable = 4'b0001; end // lb     //signed extension b,h,w select          
		12'b00001 : begin SignExtensionSelect=3'b001; ByteEnable = 4'b0011; end // lh 
		12'b00010 : begin SignExtensionSelect=3'b010; ByteEnable = 4'b1111; end// lw   
		12'b00100 : begin SignExtensionSelect=3'b100; ByteEnable = 4'b0001; end // lbu
		12'b00101 : begin SignExtensionSelect=3'b101; ByteEnable = 4'b0011; end// lhu
		default : begin SignExtensionSelect=3'b010; ByteEnable = 4'b1111; end	
		endcase				
	end	

	reg [31:0] ALUinput1;
	reg [31:0] ALUinput2; 
	wire [1:0] FORWARD_A;
	wire [1:0] FORWARD_B;
	
	////forwardingMUX
	always @* begin 
		case(FORWARD_A)
		2'b00 : ALUinput1 <= IDEX_READDATA1;
		2'b01 : ALUinput1 <= toReg;
		2'b10 : ALUinput1 <= EXMEM_ALURESULT;
		default : ALUinput1 <= 32'hffffffff;
		endcase
		
		case(FORWARD_B)
		2'b00 : ALUinput2 <= IDEX_READDATA2;
		2'b01 : ALUinput2 <= toReg;
		2'b10 : ALUinput2 <= EXMEM_ALURESULT;
		default : ALUinput2 <= 32'hffffffff;
		endcase		
	end

	////forwarding Unit
	forwarding (IDEX_RS1,IDEX_RS2, EXMEM_REGWRITE, MEMWB_REGWRITE, EXMEM_RD, MEMWB_RD, FORWARD_A,FORWARD_B);	

	////load use hazard unit
	wire TOCONTROLBITMUX,IFIDWRITEOFF,PCWRITEOFF;	
	hazarddetect(IDEX_MEMREAD,IDEX_RD,IFID_QINSTRUCTION[19:15],IFID_QINSTRUCTION[24:20],TOCONTROLBITMUX,IFIDWRITEOFF,PCWRITEOFF);
		
endmodule

module hazarddetect(IDEX_MEMREAD,IDEX_RD,IFID_RS1,IFID_RS2,TOCONTROLBITMUX,IFIDWRITEOFF,PCWRITEOFF);
	input IDEX_MEMREAD;
	input [4:0] IFID_RS1,IFID_RS2, IDEX_RD;
	output TOCONTROLBITMUX,IFIDWRITEOFF,PCWRITEOFF;
	reg TOCONTROLBITMUX,IFIDWRITEOFF,PCWRITEOFF;
	always @* begin
		if (IDEX_MEMREAD && ((IDEX_RD == IFID_RS1) || (IDEX_RD == IFID_RS2))) begin
		TOCONTROLBITMUX<=1;
		IFIDWRITEOFF<=1;
		PCWRITEOFF<=1;		
		end
		else begin
		TOCONTROLBITMUX<=0;
		IFIDWRITEOFF<=0;
		PCWRITEOFF<=0;		
		end
   end
endmodule

module forwarding ( IDEX_RS1,IDEX_RS2, EXMEM_REGWRITE, MEMWB_REGWRITE, EXMEM_RD, MEMWB_RD, FORWARD_A,FORWARD_B);	 
	input [4:0] IDEX_RS1,IDEX_RS2, EXMEM_RD, MEMWB_RD;
	input EXMEM_REGWRITE , MEMWB_REGWRITE ;
	reg  [1:0] FORWARD_A,FORWARD_B;
	output [1:0] FORWARD_A,FORWARD_B;		
	
	always  @*    begin		
		if (EXMEM_REGWRITE && (EXMEM_RD != 0) && (EXMEM_RD == IDEX_RS1)) FORWARD_A <= 2'b10;
		else					
		if (MEMWB_REGWRITE && (MEMWB_RD != 0) && (MEMWB_RD == IDEX_RS1)
			 && !(EXMEM_REGWRITE && (EXMEM_RD != 0) &&(EXMEM_RD == IDEX_RS1))) FORWARD_A <= 2'b01;
		else	FORWARD_A<=2'b00;
		
		if (EXMEM_REGWRITE && (EXMEM_RD != 0) && (EXMEM_RD == IDEX_RS2)) FORWARD_B <= 2'b10;
		else		
		if (MEMWB_REGWRITE && (MEMWB_RD != 0) && (MEMWB_RD == IDEX_RS2)
			 && !(EXMEM_REGWRITE && (EXMEM_RD != 0) &&(EXMEM_RD == IDEX_RS2))) FORWARD_B <= 2'b01;
		else   FORWARD_B<=2'b00;	 
	end	
endmodule

//IMMGEN module
module IMMGEN(qinstruction, IMMGEN);	
	input [31:0] qinstruction;
	output [31:0] IMMGEN;
	reg [31:0] IMMGEN;	
	always @* begin
		case(qinstruction[6:0])
			7'b0000011 : IMMGEN<={ {20{qinstruction[31]}},qinstruction[31:20] };  	//LOAD
			7'b0010011 : IMMGEN<={ {20{qinstruction[31]}},qinstruction[31:20] };  	//OP-IMM
			7'b0100011 : IMMGEN<={ {20{qinstruction[31]}},qinstruction[31:25],qinstruction[11:7] }; 	//STORE			
			7'b1100011 : IMMGEN<={ {20{qinstruction[31]}},qinstruction[7],qinstruction[30:25],qinstruction[11:8],1'b0 };   //BRANCH			
			7'b0110111 : IMMGEN<={ qinstruction[31:12],{12{1'b0}} };//LUI 
			7'b0010111 : IMMGEN<={ qinstruction[31:12],{12{1'b0}} };//AUIPC
			7'b1101111 : IMMGEN<={ {12{qinstruction[31]}},qinstruction[19:12],qinstruction[20],qinstruction[30:21],1'b0 }; // JAL
			7'b1100111 : IMMGEN<={ {20{qinstruction[31]}},qinstruction[31:21],1'b0 }; //JALR			
			default : IMMGEN<=32'hffffffff;  
		endcase
	end
	endmodule
	
module controller(opcode,ALUsrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp1,ALUOp0,Signal8bit,LUIsignal,JALsignal);
		input [6:0] opcode;		
		output ALUsrc;
		output MemtoReg;
		output RegWrite;
		output MemRead;
		output MemWrite;
		output Branch;
		output ALUOp1;
		output ALUOp0;
		output [1:0]LUIsignal;
		output [1:0]JALsignal;
		reg ALUsrc;
		reg MemtoReg;
		reg RegWrite;
		reg MemRead;
		reg MemWrite;
		reg Branch;
		reg ALUOp1;
		reg ALUOp0;
		reg [1:0] LUIsignal;
		reg [1:0] JALsignal;
		output [7:0] Signal8bit;//for debug
		reg [7:0] Signal8bit;		
		
		always @* begin
		LUIsignal = 2'b00;
		JALsignal = 2'b00;
		case(opcode)
			7'b0110011 : Signal8bit=8'b00100010;   //OP
			7'b0000011 : Signal8bit=8'b11110000;  	//LOAD
			7'b0100011 : Signal8bit=8'b10001000; 	//STORE			
			7'b1100011 : Signal8bit=8'b00000101;   //BRANCH
			7'b0010011 : Signal8bit=8'b10100011; //OP-IMM 			
			7'b0110111 : begin Signal8bit=8'b10100010; LUIsignal = 2'b01; end //LUI
			7'b0010111 : begin Signal8bit=8'b10100010; LUIsignal = 2'b10; end  //AUIPC
			7'b1101111 : begin Signal8bit=8'b00100000; JALsignal = 2'b01; end //JAL 
			7'b1100111 : begin Signal8bit=8'b00100000; JALsignal = 2'b10; end //JALR						
			default : Signal8bit=8'b00000000;  
		endcase
		ALUsrc=Signal8bit[7]; 
		MemtoReg=Signal8bit[6];
		RegWrite=Signal8bit[5];
		MemRead=Signal8bit[4];
		MemWrite=Signal8bit[3];
		Branch=Signal8bit[2];
		ALUOp1=Signal8bit[1];
		ALUOp0=Signal8bit[0];		
		end
endmodule

module ALU(readData1, fromMUX, Result, aluControl4bit,Zero);
	input [31:0]readData1, fromMUX;	
	input [4:0] aluControl4bit;
	output [31:0] Result;
	output Zero;
	reg Zero;
	reg [31:0]Result;
	
	always @(*) begin	
		Zero=0;  
		case(aluControl4bit) 
		4'b00000 : Result <= readData1&fromMUX; //and, andi
		4'b00001 : Result <= readData1|fromMUX; //or, ori
		4'b00010 : Result <= readData1+fromMUX; //add,addi,load,store
		4'b00011 : Result <= readData1^fromMUX; //xor ,xori		
		4'b00110 : begin                        //sub, beq
					 Result = readData1-fromMUX;
					 if(Result==0) Zero <= 1;					 
					 end
		4'b00100 : if(readData1 != fromMUX) Zero <= 1; //bne
		4'b00101 : if($signed(readData1) < $signed(fromMUX)) Zero <= 1;	 //blt
		4'b00110 : if($signed(readData1) >= $signed(fromMUX)) Zero <= 1; //bge
	    4'b00111 : if(readData1 < fromMUX) Zero <= 1;	 //bltu
		4'b01110 : if(readData1 >= fromMUX) Zero <= 1; //bgeu		
		4'b01001 : Result <= readData1<<fromMUX[4:0]; //slli,sll  
		4'b01010 : Result <= readData1>>fromMUX[4:0]; //srli ,srl
		4'b01011 : Result <= readData1>>>fromMUX[4:0]; //srai, sra		
		4'b10000 : Result <= ($signed(readData1) < $signed(fromMUX)) ? 1 : 0; //slt ,slti
		4'b10001 : Result <= (readData1 < fromMUX) ? 1 : 0; //sltu ,sltiu				
		default : Result <= 32'hffffffff;	
		endcase				
	end		
endmodule

module ALU_controller(ALUOp,Funct7, Funct3,  ALUcontrol4bit,LUIsignal);
	input [6:0] Funct7;
	input [2:0] Funct3;
	input [1:0] ALUOp;
	input [1:0]LUIsignal;
	output [4:0] ALUcontrol4bit;
	
	reg [4:0] ALUcontrol4bit;
	reg [5:0] temp;	
	always @* begin
	temp = {{ALUOp},{Funct7[5]},{Funct3}};	
	
	casex(temp)		
		6'b01x000 : ALUcontrol4bit <= 5'b00110; //beq		
		6'b01x001 : ALUcontrol4bit <= 5'b00100; //bne
		6'b01x100 : ALUcontrol4bit <= 5'b00101; //blt
		6'b01x101 : ALUcontrol4bit <= 5'b00110; //bge
		6'b01x110 : ALUcontrol4bit <= 5'b00111; //bltu
		6'b01x111 : ALUcontrol4bit <= 5'b01110; //bgeu		 
		6'b00xxxx : ALUcontrol4bit <= 5'b00010; //lw ,sw 
		6'b1xx010 : ALUcontrol4bit <= 5'b10000; //slt ,slti
		6'b1xx011 : ALUcontrol4bit <= 5'b10001; //sltu ,sltiu
		6'b1xx100 : ALUcontrol4bit <= 5'b00011; //xor,xori		
		6'b1xx110 : ALUcontrol4bit <= 5'b00001; //or, ori
		6'b1xx111 : ALUcontrol4bit <= 5'b00000; //and, andi
		6'b1x0001 : ALUcontrol4bit <= 5'b01001; //sll,slli  
		6'b1x0101 : ALUcontrol4bit <= 5'b01010; //srl, srli	
	    6'b1x1101 : ALUcontrol4bit <= 5'b01011; //sra, srai
		6'b100000 : ALUcontrol4bit <= 5'b00010;	//add
	    6'b101000 : ALUcontrol4bit <= 5'b00110; //sub	
		6'b11x000 : ALUcontrol4bit <= 5'b00010;	//addi
		default : ALUcontrol4bit <= 5'b11111;
	endcase
	
	if(LUIsignal) ALUcontrol4bit <= 5'b00010; //lui , auipc
	end
endmodule







	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	