`include "macro.vh"

module mycpu_core(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_req,
    output wire        inst_sram_wr,
    output wire [ 1:0] inst_sram_size,
    output wire [ 3:0] inst_sram_wstrb,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire        inst_sram_addr_ok,
    input  wire        inst_sram_data_ok,
    input  wire [31:0] inst_sram_rdata,

    // data sram interface
    output wire        data_sram_req,
    output wire        data_sram_wr,
    output wire [ 1:0] data_sram_size,
    output wire [ 3:0] data_sram_wstrb,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire        data_sram_addr_ok,
    input  wire        data_sram_data_ok,
    input  wire [31:0] data_sram_rdata,

    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,

    //for icache
    output wire [31:0]  inst_icache_vaddr
);

reg         reset;
always @(posedge clk) begin
    reset <= ~resetn;
end

wire [31:0] seq_pc;
wire [31:0] nextpc;
wire        br_taken;
wire [31:0] br_target;
wire [31:0] inst;
reg  [31:0] pc;

wire [18:0] alu_op;
wire [ 1:0] mul_op;
wire [ 1:0] div_op;
wire [ 3:0] load_op;//new
wire [ 2:0] store_op;//new
wire        tmr_op;//new
wire [ 4:0] tlb_op;
reg  [18:0] ex_alu_op;
reg  [ 1:0] ex_mul_op;
reg  [ 1:0] ex_div_op;
reg  [ 3:0] ex_load_op;
reg  [ 2:0] ex_store_op;
reg  [ 4:0] ex_tlb_op;
reg  [ 3:0] mem_load_op;
reg  [ 2:0] mem_store_op;
reg  [ 4:0] mem_tlb_op;
reg  [ 4:0] wb_tlb_op;
reg         ex_tmr_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        res_from_alu;//new
wire        res_from_mul;
wire        res_from_div;
wire        res_from_csr;//new
wire        res_from_tmr;//new
wire        res_from_mem;
reg         ex_res_from_alu;
reg         ex_res_from_mul;
reg         ex_res_from_div;
reg         ex_res_from_csr;
reg         ex_res_from_tmr;
reg         ex_res_from_mem;

wire        dst_is_r1;
wire        dst_is_rj;//new
wire        gr_we;
wire        mem_en;
wire        mem_we;
wire        src_reg_is_rj;
wire        src_reg_is_rd;
wire        src_reg_is_rkd;
wire [4: 0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
reg  [31:0] ex_rkd_value;
wire [31:0] imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] op_14_10;
wire [ 4:0] op_9_5;
wire [ 4:0] op_4_0;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;
wire [31:0] op_14_10_d;
wire [31:0] op_9_5_d;
wire [31:0] op_4_0_d;

wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;

wire        inst_ld_w;
wire        inst_ld_b;
wire        inst_ld_h;
wire        inst_ld_bu;
wire        inst_ld_hu;

wire        inst_st_w;
wire        inst_st_b;
wire        inst_st_h;

wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;

wire        inst_lu12i_w;

wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_pcaddu12i;
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_mod_w;
wire        inst_div_wu;
wire        inst_mod_wu;

wire        inst_csrrd;
wire        inst_csrwr;
wire        inst_csrxchg;
wire        inst_ertn;
wire        inst_syscall;

wire        inst_rdcntvl_w;
wire        inst_rdcntvh_w;
wire        inst_rdcntid_w;

wire        need_ui5;
wire        need_ui12;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire        id_rf_we;
wire [ 4:0] id_rf_waddr;
reg         ex_rf_we;
reg         mem_rf_we;
reg         wb_rf_we;
reg  [ 4:0] ex_rf_waddr;
reg  [ 4:0] mem_rf_waddr;
reg  [ 4:0] wb_rf_waddr;
reg  [31:0] wb_rf_wdata;

wire        rf_re1;
wire        rf_re2;
wire [ 2:0] rf_raddr1_eq_waddr;
wire [ 2:0] rf_raddr2_eq_waddr;
wire [ 2:0] rf_rhazard1;
wire [ 2:0] rf_rhazard2;
wire [31:0] rf_rforward1;
wire [31:0] rf_rforward2;

wire        load_use;

wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;
wire        alu_complete;
reg  [31:0] ex_alu_src1;
reg  [31:0] ex_alu_src2;

wire [31:0] mul_result ;
wire [31:0] div_result ;

wire [31:0] mem_result ;
wire [31:0] csr_result ;
wire [31:0] tmr_result ;

wire [31:0] ex_result;
reg  [31:0] ex_result_out;

wire [31:0] final_result;

wire [31:0] id_data_sram_wdata;
reg  [31:0] ex_data_sram_wdata;
wire [31:0] mem_data;
wire [ 7:0] mem_data_b;
wire [15:0] mem_data_h;

reg  [31:0] id_pc;
reg  [31:0] ex_pc;
reg  [31:0] mem_pc;
reg  [31:0] wb_pc;

reg  [31:0] id_inst;

wire [31:0] ex_vaddr;
reg  [31:0] mem_vaddr;
reg  [31:0] wb_vaddr;

reg         pf_valid_r;
reg         if_valid_r;
reg         id_valid_r;
reg         ex_valid_r;
reg         mem_valid_r;
reg         wb_valid_r;

reg         id_ex_in;
reg         ex_ex_in;
reg         mem_ex_in;
reg         wb_ex_in;

reg  [ 5:0] id_ecode_in;
reg  [ 5:0] ex_ecode_in;
reg  [ 5:0] mem_ecode_in;
reg  [ 5:0] wb_ecode_in;
wire [ 5:0] if_ecode;
wire [ 5:0] id_ecode;
wire [ 5:0] ex_ecode;
wire [ 5:0] mem_ecode;
wire [ 5:0] wb_ecode;

wire [ 8:0] wb_esubcode;

reg         ex_ertn;
reg         mem_ertn;
reg         wb_ertn;

wire        id_csr_we;
reg         ex_csr_we;
reg         mem_csr_we;
reg         wb_csr_we;
wire        csr_we;
wire [13:0] id_csr_num;
reg  [13:0] ex_csr_num;
reg  [13:0] mem_csr_num;
reg  [13:0] wb_csr_num;
wire [13:0] csr_wnum;
wire [13:0] csr_rnum;
wire [31:0] id_csr_wmask;
reg  [31:0] ex_csr_wmask;
reg  [31:0] mem_csr_wmask;
reg  [31:0] wb_csr_wmask;
wire [31:0] csr_wmask;
wire [31:0] id_csr_wvalue;
reg  [31:0] ex_csr_wvalue;
reg  [31:0] mem_csr_wvalue;
reg  [31:0] wb_csr_wvalue;
wire [31:0] csr_wvalue;

wire [ 4:0] invtlb_op;
wire [ 9:0] invtlb_asid;
wire [18:0] invtlb_vppn;
reg  [ 4:0] ex_invtlb_op;
reg  [ 9:0] ex_invtlb_asid;
reg  [18:0] ex_invtlb_vppn;

wire [31:0] csr_rvalue;
wire [31:0] ex_ra;
wire [31:0] ex_entry;

reg         inst_buf_valid;
reg  [31:0] inst_buf;
reg         inst_discard;
reg         wb_ex_r;
reg         ertn_flush_r;
reg         br_taken_r;
reg  [31:0] ex_entry_r;
reg  [31:0] ex_ra_r;
reg  [31:0] br_target_r;

wire [31:0] if_inst;

wire [ 4:0] ex_tlb_op_out;
wire [ 4:0] wb_tlb_op_out;
wire [ 4:0] ex_invtlb_op_out;

wire [31:0] inst_sram_vaddr;
wire [31:0] inst_sram_paddr;
wire        has_tlbr_inst_sram;
wire        has_pif_inst_sram;
wire        has_ppi_inst_sram;

wire [31:0] data_sram_vaddr;
wire [31:0] data_sram_paddr;
wire        has_tlbr_data_sram;
wire        has_pil_data_sram;
wire        has_pis_data_sram;
wire        has_pme_data_sram;
wire        has_ppi_data_sram;

always @(posedge clk) begin
    if(reset) begin
        pf_valid_r       <= 1'b0;
    end
    else begin
        if(~reset & if_allow_in & ~br_stall) begin
            pf_valid_r   <= 1'b1;
        end
        else if(pf_ready_go & if_allow_in) begin
            pf_valid_r   <= 1'b0;
        end
		else begin
			pf_valid_r	<= pf_valid;
		end
    end
end
assign pf_valid     = pf_valid_r & ~br_taken & ~wb_ex & ~ertn_flush;
assign pf_valid_out = pf_valid & pf_ready_go & if_allow_in;

always @(posedge clk) begin
    if(reset) begin
        if_valid_r          <= 1'b0;
    end
    else begin
        if(pf_ready_go & if_allow_in) begin
            if_valid_r      <= ~if_cancel;
        end
        else if(if_ready_go & id_allow_in) begin
            if_valid_r      <= 1'b0;
        end
        else begin
            if_valid_r      <= if_valid;
        end
    end
end
assign if_valid     = if_valid_r & ~br_taken & ~wb_ex & ~ertn_flush & ~id_refetch;
assign if_valid_out = if_valid & if_ready_go & id_allow_in;

always @(posedge clk) begin
    if(reset) begin
        id_valid_r        <= 1'b0;
    end
    else begin
        if(if_ready_go & id_allow_in) begin
            id_valid_r    <= if_valid;
        end
        else if(id_ready_go & ex_allow_in) begin
            id_valid_r    <= 1'b0;
        end 
        else begin
            id_valid_r    <= id_valid;
        end
    end
end
assign id_valid     = id_valid_r & ~wb_ex & ~ertn_flush;
assign id_valid_out = id_valid & id_ready_go & ex_allow_in;

always @(posedge clk) begin
    if(reset) begin
        ex_valid_r        <= 1'b0;
    end
    else begin
        if(id_ready_go & ex_allow_in) begin
            ex_valid_r    <= id_valid;
        end
        else if(ex_ready_go & mem_allow_in) begin
            ex_valid_r    <= 1'b0;
        end
        else begin
            ex_valid_r    <= ex_valid;
        end 
    end
end
assign ex_valid     = ex_valid_r & ~wb_ex & ~ertn_flush & ~mem_ex & ~(mem_ertn & mem_valid);
assign ex_valid_out = ex_valid & ex_ready_go & mem_allow_in;

always @(posedge clk) begin
    if(reset) begin
        mem_valid_r       <= 1'b0;
    end
    else begin
        if(ex_ready_go & mem_allow_in) begin
            mem_valid_r   <= ex_valid;
        end
        else if(mem_ready_go & wb_allow_in) begin
            mem_valid_r   <= 1'b0;
        end
        else begin
            mem_valid_r   <= mem_valid;
        end
    end
end
assign mem_valid    = mem_valid_r & ~wb_ex & ~ertn_flush;
assign mem_valid_out = mem_valid & mem_ready_go & wb_allow_in;

always @(posedge clk) begin
    if(reset) begin
        wb_valid_r        <= 1'b0;
    end
    else begin
        if(mem_ready_go & wb_allow_in) begin
            wb_valid_r    <= mem_valid;
        end
        else if(wb_ready_go) begin
            wb_valid_r    <= 1'b0;
        end
    end
end
assign wb_valid     = wb_valid_r;

always @(posedge clk) begin
    if(reset) begin
        inst_buf_valid <= 1'b0;
        inst_buf <= 32'b0;
    end
    else begin
        if(if_ready_go & id_allow_in | if_cancel) begin
            inst_buf_valid <= 1'b0;
        end
        else if(inst_sram_data_ok & ~inst_discard) begin
            inst_buf_valid <= 1'b1;
            inst_buf <= inst;
        end
    end
end

always @(posedge clk) begin
    if(reset) begin
        inst_discard <= 1'b0;
    end
    else begin
        if(inst_sram_data_ok) begin
            inst_discard <= 1'b0;
        end
        else if(if_cancel & ~if_allow_in & ~if_ready_go) begin//有在等待有效指令才discard
            inst_discard <= 1'b1;
        end
    end
end

reg block_pf;
reg refetch_r;
reg [31:0] refetch_pc_r;
wire ex_refetch;
wire wb_refetch;

always @(posedge clk) begin
    if(reset) begin
        block_pf <= 1'b0;//for refetch, block when there is an inst which will change tlb in id
    end//and unblock when tlb has been changed in ex or wb
    else begin
        if(id_refetch) begin
            block_pf <= 1'b1;
        end
        else if(ex_refetch | wb_refetch) begin
            block_pf <= 1'b0;
        end
    end
end

assign		pf_ready_go = inst_sram_req & inst_sram_addr_ok;
assign      if_ready_go = inst_sram_data_ok | inst_buf_valid;
assign      id_ready_go = ~load_use & ~block_id | wb_ex | ertn_flush;//1'b1;
assign      ex_ready_go = ~block_ex & alu_complete & (~data_sram_req | data_sram_req & data_sram_addr_ok);
assign      mem_ready_go = ~(|mem_load_op) & ~(|mem_store_op) | data_sram_data_ok | mem_ex;
assign      wb_ready_go = 1'b1;

assign      pf_allow_in = ~block_pf;
assign      if_allow_in = ~if_valid_r & ~inst_discard | if_ready_go & id_allow_in;
assign      id_allow_in = ~id_valid | ex_allow_in & id_ready_go;//1'b1;
assign      ex_allow_in = ~ex_valid | ex_ready_go & mem_allow_in;//~div;//
assign      mem_allow_in = ~mem_valid | mem_ready_go & wb_allow_in;
assign      wb_allow_in = 1'b1;

always @(posedge clk) begin
    if (reset) begin
        id_inst <= 32'b0;
		ex_rf_we <= 1'b0;
		mem_rf_we <= 1'b0;
		wb_rf_we <= 1'b0;
		ex_load_op <= 4'b0;
		ex_store_op <= 3'b0;
        mem_load_op <= 4'b0;
        mem_store_op <= 3'b0;
        ex_tlb_op <= 5'b0;
        mem_tlb_op <= 5'b0;
        wb_tlb_op <= 5'b0;
        
        id_ex_in <= 1'b0;
        ex_ex_in <= 1'b0;
        mem_ex_in <= 1'b0;
        wb_ex_in <= 1'b0;
    end
    else  begin
        if (if_ready_go & id_allow_in) begin

            id_inst <= if_inst;
            id_pc <= pc;

            id_ex_in <= if_ex;
            id_ecode_in <= if_ecode;
        end
        if (id_ready_go & ex_allow_in) begin

            ex_pc <= id_pc;

            ex_alu_op <= alu_op;
            ex_alu_src1 <= alu_src1;
            ex_alu_src2 <= alu_src2;
            ex_mul_op <= mul_op;
            ex_div_op <= div_op;
            ex_load_op <= load_op;
            ex_store_op <= store_op;
            ex_tmr_op <= tmr_op;
            ex_tlb_op <= tlb_op;

            ex_res_from_alu <= res_from_alu;
            ex_res_from_mul <= res_from_mul;
            ex_res_from_div <= res_from_div;
            ex_res_from_csr <= res_from_csr;
            ex_res_from_tmr <= res_from_tmr;
            ex_res_from_mem <= res_from_mem;

            ex_rf_we <= id_rf_we;
            ex_rf_waddr <= id_rf_waddr;

            ex_data_sram_wdata <= id_data_sram_wdata;

            ex_csr_we <= id_csr_we;
            ex_csr_num <= id_csr_num;
            ex_csr_wmask <= id_csr_wmask;
            ex_csr_wvalue <= id_csr_wvalue;

            ex_invtlb_op <= invtlb_op;
            ex_invtlb_asid <= invtlb_asid;
            ex_invtlb_vppn <= invtlb_vppn;

            ex_ertn <= inst_ertn;

            ex_ex_in <= id_ex;
            ex_ecode_in <= id_ecode;
        end
        if (ex_ready_go & mem_allow_in) begin

            mem_pc <= ex_pc;
            mem_vaddr <= ex_vaddr;

            mem_rf_we <= ex_rf_we;
            mem_rf_waddr <= ex_rf_waddr;

            ex_result_out <= ex_result;

            mem_load_op <= ex_load_op;
            mem_store_op <= ex_store_op;
            mem_tlb_op <= ex_tlb_op;

            mem_csr_we <= ex_csr_we;
            mem_csr_num <= ex_csr_num;
            mem_csr_wmask <= ex_csr_wmask;
            mem_csr_wvalue <= ex_csr_wvalue;

            mem_ertn <= ex_ertn;

            mem_ex_in <= ex_ex;
            mem_ecode_in <= ex_ecode;
        end
        if (mem_ready_go & wb_allow_in) begin

            wb_pc <= mem_pc;
            wb_vaddr <= mem_vaddr;

            wb_rf_we <= mem_rf_we;
            wb_rf_waddr <= mem_rf_waddr;
            wb_rf_wdata <= final_result;

            wb_tlb_op <= mem_tlb_op;

            wb_csr_we <= mem_csr_we;
            wb_csr_num <= mem_csr_num;
            wb_csr_wmask <= mem_csr_wmask;
            wb_csr_wvalue <= mem_csr_wvalue;

            wb_ertn <= mem_ertn;

            wb_ex_in <= mem_ex;
            wb_ecode_in <= mem_ecode;
        end
    end
end

//IF
assign if_cancel = wb_ex | ertn_flush | br_taken | id_refetch;

always @(posedge clk) begin
    if(reset) begin
        {wb_ex_r, ertn_flush_r, br_taken_r} <= 3'b0;
        {ex_entry_r, ex_ra_r, br_target_r} <= {3{32'b0}};
        refetch_r <= 1'b0;
        refetch_pc_r <= 32'b0;
    end
    // 若对应地址已经获得了来自指令SRAM的ok，后续nextpc不再从寄存器中取
    else if(pf_ready_go & ~if_cancel) begin
        {wb_ex_r, ertn_flush_r, br_taken_r} <= 3'b0;
        refetch_r <= 1'b0;
    end
    else if(id_refetch) begin
        refetch_r <= 1'b1;
        refetch_pc_r <= if_valid_r ? pc : nextpc;
    end
    // 当前仅当遇到fs_cancel时未等到pf_ready_go，需要将cancel相关信号存储在寄存器
    else if(wb_ex) begin
        ex_entry_r <= ex_entry;
        wb_ex_r <= 1'b1;
    end
    else if(ertn_flush) begin
        ex_ra_r <= ex_ra;
        ertn_flush_r <= 1'b1;
    end    
    else if(br_taken) begin
        br_target_r <= br_target;
        br_taken_r <= 1'b1;
    end
    /*
    else if(br_taken_r & has_adef) begin
        br_taken_r <= 1'b0;
    end*/
end

assign seq_pc       = pc + 3'h4;
assign nextpc       = refetch_r ? refetch_pc_r :
                      wb_ex_r ? ex_entry_r : wb_ex ? ex_entry :
                      ertn_flush_r ? ex_ra_r : ertn_flush ? ex_ra :
                      br_taken_r ? br_target_r : br_taken ? br_target : seq_pc;

assign inst_icache_vaddr = nextpc;
assign inst_sram_vaddr = nextpc;

always @(posedge clk) begin
    if (reset) begin
        pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset 
    end
    else begin
        if(pf_ready_go & if_allow_in) begin
            pc <= nextpc;
        end
    end
end

assign inst = inst_sram_rdata;
assign if_inst = inst_buf_valid ? inst_buf : inst;
assign inst_sram_req = ~reset & if_allow_in & ~br_stall & ~block_pf;
assign inst_sram_wr = 1'b0;
assign inst_sram_size = 2'b10;
assign inst_sram_wstrb = 4'b0;

assign inst_sram_addr = inst_sram_paddr;
assign inst_sram_wdata = 32'b0;

assign has_adef         = |pc[1:0];
assign if_ex            = has_adef | has_tlbr_inst_sram | has_pif_inst_sram | has_ppi_inst_sram;
assign if_ecode         = {6{has_adef}} & `ECODE_ADE | {6{has_tlbr_inst_sram}} & `ECODE_TLBR
                        | {6{has_pif_inst_sram}} & `ECODE_PIF | {6{has_ppi_inst_sram}} & `ECODE_PPI;

//ID

assign op_31_26  = id_inst[31:26];
assign op_25_22  = id_inst[25:22];
assign op_21_20  = id_inst[21:20];
assign op_19_15  = id_inst[19:15];
assign op_14_10  = id_inst[14:10];
assign op_9_5    = id_inst[ 9: 5];
assign op_4_0    = id_inst[ 4: 0];

assign rd   = id_inst[ 4: 0];
assign rj   = id_inst[ 9: 5];
assign rk   = id_inst[14:10];

assign i12  = id_inst[21:10];
assign i20  = id_inst[24: 5];
assign i16  = id_inst[25:10];
assign i26  = {id_inst[ 9: 0], id_inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));
decoder_5_32 u_dec4(.in(op_14_10 ), .out(op_14_10_d ));
decoder_5_32 u_dec5(.in(op_9_5   ), .out(op_9_5_d   ));
decoder_5_32 u_dec6(.in(op_4_0   ), .out(op_4_0_d   ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_ld_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_ld_bu  = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_ld_hu  = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_st_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_st_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h5];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_blt    = op_31_26_d[6'h18];
assign inst_bge    = op_31_26_d[6'h19];
assign inst_bltu   = op_31_26_d[6'h1a];
assign inst_bgeu   = op_31_26_d[6'h1b];

assign inst_lu12i_w= op_31_26_d[6'h05] & ~id_inst[25];
assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_mul_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];
assign inst_pcaddu12i = op_31_26_d[6'h07] & ~id_inst[25];

assign inst_csrrd       = op_31_26_d[6'h01] & ~id_inst[25] & ~id_inst[24] & op_9_5_d[5'h00];
assign inst_csrwr       = op_31_26_d[6'h01] & ~id_inst[25] & ~id_inst[24] & op_9_5_d[5'h01];
assign inst_csrxchg     = op_31_26_d[6'h01] & ~id_inst[25] & ~id_inst[24] & ~op_9_5_d[5'h00] & ~op_9_5_d[5'h01];
assign inst_ertn        = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & op_14_10_d[5'h0e] & op_9_5_d[5'h00] & op_4_0_d[5'h00];
assign inst_syscall     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break       = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];

assign inst_rdcntvl_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & op_14_10_d[5'h18] & op_9_5_d[5'h00];
assign inst_rdcntvh_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & op_14_10_d[5'h19] & op_9_5_d[5'h00];
assign inst_rdcntid_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & op_14_10_d[5'h18] & op_4_0_d[5'h00];

assign inst_tlbsrch     = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & op_14_10_d[5'h0a] & op_9_5_d[5'h00] & op_4_0_d[5'h00];
assign inst_tlbrd       = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & op_14_10_d[5'h0b] & op_9_5_d[5'h00] & op_4_0_d[5'h00];
assign inst_tlbwr       = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & op_14_10_d[5'h0c] & op_9_5_d[5'h00] & op_4_0_d[5'h00];
assign inst_tlbfill     = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & op_14_10_d[5'h0d] & op_9_5_d[5'h00] & op_4_0_d[5'h00];
assign inst_invtlb      = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h13]
                        & (|op_4_0_d[6:0]);

assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                    | inst_st_w | inst_st_b | inst_st_h 
                    | inst_jirl | inst_bl | inst_pcaddu12i;
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltui;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or |inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_slli_w | inst_sll_w;
assign alu_op[ 9] = inst_srli_w | inst_srl_w;
assign alu_op[10] = inst_srai_w | inst_sra_w;
assign alu_op[11] = inst_lu12i_w;
assign alu_op[12] = inst_mul_w ;
assign alu_op[13] = inst_mulh_w;
assign alu_op[14] = inst_mulh_wu;
assign alu_op[15] = inst_div_w ;
assign alu_op[16] = inst_div_wu;
assign alu_op[17] = inst_mod_w ;
assign alu_op[18] = inst_mod_wu;

assign mul_op = {inst_mulh_wu, inst_mulh_w | inst_mulh_wu};
assign div_op = {inst_div_wu | inst_mod_wu, inst_mod_w | inst_mod_wu};

assign load_op = {inst_ld_bu | inst_ld_hu, inst_ld_w, inst_ld_h | inst_ld_hu, inst_ld_b | inst_ld_bu};
assign store_op = {inst_st_w, inst_st_h, inst_st_b};

assign tmr_op     = inst_rdcntvh_w;

assign tlb_op = {inst_invtlb, inst_tlbfill, inst_tlbwr, inst_tlbrd, inst_tlbsrch};

assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_ui12  =  inst_andi | inst_ori | inst_xori;
assign need_si12  =  inst_addi_w | inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu | 
                        inst_st_w | inst_st_b | inst_st_h | inst_slti | inst_sltui;
assign need_si16  =  inst_jirl | inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu;
assign need_si20  =  inst_lu12i_w | inst_pcaddu12i;
assign need_si26  =  inst_b | inst_bl;
assign src2_is_4  =  inst_jirl | inst_bl;

assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
             need_ui5 || need_si12 ?{{20{i12[11]}}, i12[11:0]} :
/*need_ui12*/{20'b0,i12[11:0]} ;//TODO

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rj = ~inst_b & ~inst_bl & ~inst_lu12i_w & ~inst_csrrd & ~inst_csrwr & ~inst_ertn & ~inst_syscall & ~inst_rdcntvl_w & ~inst_rdcntvh_w & ~inst_rdcntid_w
                        &~inst_tlbsrch & ~inst_tlbrd & ~inst_tlbwr & ~inst_tlbfill;
assign src_reg_is_rd = inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu | inst_st_w | inst_st_b | inst_st_h
                     | inst_csrwr | inst_csrxchg;
assign src_reg_is_rkd = ~(inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w | inst_andi | inst_ori | inst_xori | inst_slti | inst_sltui)
                         & ~inst_bl & ~inst_b & ~inst_lu12i_w & ~inst_jirl & ~inst_csrrd & ~inst_ertn & ~inst_syscall & ~inst_rdcntvl_w & ~inst_rdcntvh_w & ~inst_rdcntid_w
                         & ~inst_tlbsrch & ~inst_tlbrd & ~inst_tlbwr & ~inst_tlbfill;

assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;

assign src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_ld_w   |
                       inst_ld_b   |
                       inst_ld_h   |
                       inst_ld_bu  |
                       inst_ld_hu  |
                       inst_st_w   |
                       inst_st_b   |
                       inst_st_h   |
                       inst_lu12i_w|
                       inst_jirl   |
                       inst_bl     |
                       inst_slti   |
                       inst_sltui  |
                       inst_andi   |
                       inst_ori    |
                       inst_xori   |
                       inst_pcaddu12i;

assign res_from_alu = gr_we & ~res_from_csr & ~res_from_tmr & ~res_from_mem;
//assign res_from_mul = inst_mul_w | inst_mulh_w | inst_mulh_wu;
//assign res_from_div = inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu;
assign res_from_csr = inst_csrrd | inst_csrwr | inst_csrxchg | inst_rdcntid_w;
assign res_from_tmr = inst_rdcntvl_w | inst_rdcntvh_w;
assign res_from_mem  = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
assign dst_is_r1     = inst_bl;
assign dst_is_rj     = inst_rdcntid_w;
assign gr_we         = ~inst_st_w & ~inst_st_b & ~inst_st_h & ~inst_beq & ~inst_bne & ~inst_b 
                        & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu & ~inst_ertn & ~inst_syscall
                        & ~inst_tlbsrch & ~inst_tlbrd & ~inst_tlbwr & ~inst_tlbfill & ~inst_invtlb;
assign mem_we        = inst_st_w | inst_st_b | inst_st_h;
assign dest          = dst_is_r1 ? 5'd1 : dst_is_rj ? rj : rd;

assign id_rf_we = gr_we & id_valid;
assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;
assign id_rf_waddr = dest;

assign rj_value  = rf_rhazard1 ? rf_rforward1 : rf_rdata1;
assign rkd_value = rf_rhazard2 ? rf_rforward2 : rf_rdata2;

wire [34:0] br_cmp;
assign br_cmp = {rj_value[31], rj_value, 1'b1} + {~rkd_value[31], ~rkd_value, 1'b1};
assign rj_eq_rd = (rj_value == rkd_value);
assign rj_lt_rd = br_cmp[33];
assign rj_ltu_rd = ~br_cmp[34];

assign br_taken = (   inst_beq  &&  rj_eq_rd
                   || inst_bne  && !rj_eq_rd
                   || inst_blt  &&  rj_lt_rd
                   || inst_bge  && !rj_lt_rd
                   || inst_bltu &&  rj_ltu_rd
                   || inst_bgeu && ~rj_ltu_rd
                   || inst_jirl
                   || inst_bl
                   || inst_b
                  ) && id_valid & ~(load_use | block_id);
assign br_target = (inst_beq || inst_bne || inst_bl || inst_b || inst_blt || inst_bge || inst_bltu || inst_bgeu) ? (id_pc + br_offs) :
                                                   /*inst_jirl*/ (rj_value + jirl_offs);
assign br_stall = (load_use | block_id) & br_taken;//

assign alu_src1 = src1_is_pc  ? id_pc[31:0] : rj_value;
assign alu_src2 = src2_is_imm ? imm : rkd_value;

assign id_data_sram_wdata = rkd_value;

assign id_csr_we        = inst_csrwr | inst_csrxchg;
assign id_csr_num       = inst_rdcntid_w ? `CSR_TID : id_inst[23:10];
assign id_csr_wmask     = {32{inst_csrwr}} | {32{inst_csrxchg}} & rj_value;
assign id_csr_wvalue    = rkd_value;

assign invtlb_op = rd;
assign invtlb_asid = rj_value[9:0];
assign invtlb_vppn = rkd_value[31:13];

assign rf_re1 = src_reg_is_rj & (|rf_raddr1);
assign rf_re2 = src_reg_is_rkd & (|rf_raddr2);
assign rf_raddr1_eq_waddr[0] = (rf_raddr1 == ex_rf_waddr) & ex_rf_we & ex_valid;
assign rf_raddr1_eq_waddr[1] = (rf_raddr1 == mem_rf_waddr) & mem_rf_we & mem_valid;
assign rf_raddr1_eq_waddr[2] = (rf_raddr1 == wb_rf_waddr) & wb_rf_we & wb_valid;
assign rf_raddr2_eq_waddr[0] = (rf_raddr2 == ex_rf_waddr) & ex_rf_we & ex_valid;
assign rf_raddr2_eq_waddr[1] = (rf_raddr2 == mem_rf_waddr) & mem_rf_we & mem_valid;
assign rf_raddr2_eq_waddr[2] = (rf_raddr2 == wb_rf_waddr) & wb_rf_we & wb_valid;
assign rf_rhazard1 = (|rf_raddr1_eq_waddr) & rf_re1;
assign rf_rhazard2 = (|rf_raddr2_eq_waddr) & rf_re2;
assign rf_rforward1   = rf_raddr1_eq_waddr[0] ? ex_result : (rf_raddr1_eq_waddr[1] ? final_result : rf_wdata);
assign rf_rforward2   = rf_raddr2_eq_waddr[0] ? ex_result : (rf_raddr2_eq_waddr[1] ? final_result : rf_wdata);

assign load_use = (rf_re1 & rf_raddr1_eq_waddr[0] | rf_re2 & rf_raddr2_eq_waddr[0]) & ex_res_from_mem |
                    (rf_re1 & rf_raddr1_eq_waddr[1] | rf_re2 & rf_raddr2_eq_waddr[1]) & (|mem_load_op) & ~mem_ready_go;

assign block_id = csr_ex_block_id | csr_mem_block_id;
assign csr_ex_block_id = res_from_csr & ex_csr_we & (id_csr_num == ex_csr_num) & (|id_csr_num) & ex_valid;
assign csr_mem_block_id = res_from_csr & mem_csr_we & (id_csr_num == mem_csr_num) & (|id_csr_num) & mem_valid;

assign id_refetch = (inst_tlbrd || inst_tlbwr || inst_tlbfill || inst_invtlb || (inst_csrwr || inst_csrxchg)
                 && (id_csr_num == `CSR_CRMD || id_csr_num == `CSR_ASID)) && id_valid;

assign has_sys          = inst_syscall;
assign has_brk          = inst_break;
assign has_ine          = ~(inst_add_w  | inst_sub_w   | inst_slt       | inst_sltu      | inst_nor       |
                            inst_and    | inst_or      | inst_xor       | inst_slli_w    | inst_srli_w    |
                            inst_srai_w | inst_addi_w  | inst_ld_w      | inst_st_w      | inst_jirl      |
                            inst_b      | inst_bl      | inst_beq       | inst_bne       | inst_lu12i_w   |
                            inst_slti   | inst_sltui   | inst_andi      | inst_ori       | inst_xori      |
                            inst_sll_w  | inst_srl_w   | inst_sra_w     | inst_pcaddu12i | inst_mul_w     |
                            inst_mulh_w | inst_mulh_wu | inst_div_w     | inst_div_wu    | inst_mod_w     |
                            inst_mod_wu | inst_blt     | inst_bge       | inst_bltu      | inst_bgeu      |
                            inst_ld_b   | inst_ld_h    | inst_st_b      | inst_st_h      | inst_ld_bu     |
                            inst_ld_hu  | inst_csrrd   | inst_csrwr     | inst_csrxchg   | inst_syscall   |
                            inst_ertn   | inst_break   | inst_rdcntid_w | inst_rdcntvh_w | inst_rdcntvl_w |
                            inst_tlbsrch| inst_tlbrd   | inst_tlbwr     | inst_tlbfill   | inst_invtlb);

assign id_ex            = id_ex_in | has_int | has_sys | has_brk | has_ine;
assign id_ecode         = has_int ? `ECODE_INT : 
                          id_ex_in ? id_ecode_in : 
                          {6{has_sys}} & `ECODE_SYS | {6{has_brk}} & `ECODE_BRK | {6{has_ine}} & `ECODE_INE;

//EX

assign ex_result = {32{ex_res_from_alu}} & alu_result | 
                   {32{ex_res_from_csr}} & csr_result | {32{ex_res_from_tmr}} & tmr_result;

wire [3:0] data_sram_mask;
assign data_sram_mask  = {alu_result[1] & alu_result[0], alu_result[1] & ~alu_result[0], ~alu_result[1] & alu_result[0], ~alu_result[1] & ~alu_result[0]};
assign data_sram_req   = ((|ex_store_op) || (|ex_load_op)) & ex_valid & mem_allow_in & ~ex_ex;
assign data_sram_wr    = (|ex_store_op);
assign data_sram_size  = {ex_store_op[2], ex_store_op[1]};
assign data_sram_wstrb = ({4{ex_store_op[2]}} | {4{ex_store_op[1]}} & {{2{data_sram_mask[2]}}, {2{data_sram_mask[0]}}} | {4{ex_store_op[0]}} & data_sram_mask) & {4{ex_valid}};

assign data_sram_vaddr = alu_result;
assign data_sram_addr = data_sram_paddr;
assign data_sram_wdata = {4{{8{ex_store_op[0]}} & ex_data_sram_wdata[7:0]}} | 
                         {2{{16{ex_store_op[1]}} & ex_data_sram_wdata[15:0]}} | 
                         {32{ex_store_op[2]}} & ex_data_sram_wdata;

assign csr_rnum         = ex_csr_num;
assign csr_result       = csr_rvalue;

assign ex_tlb_op_out = ex_tlb_op & (`TLB_INST_TLBSRCH_MASK | `TLB_INST_INVTLB_MASK) & {5{ex_valid & mem_allow_in}};
assign ex_invtlb_op_out = ex_invtlb_op & {5{ex_valid & ex_ready_go & mem_allow_in}};

assign ex_refetch = ex_tlb_op[`TLB_INST_INVTLB] & ex_valid;

assign block_ex = csr_mem_block_ex | csr_wb_block_ex;
assign csr_mem_block_ex = (ex_tlb_op[`TLB_INST_TLBSRCH] | ex_tlb_op[`TLB_INST_INVTLB])
                        & (mem_csr_we & (mem_csr_num == `CSR_ASID || mem_csr_num == `CSR_TLBEHI) || mem_tlb_op[`TLB_INST_TLBRD])
                        & mem_valid;
assign csr_wb_block_ex = (ex_tlb_op[`TLB_INST_TLBSRCH] | ex_tlb_op[`TLB_INST_INVTLB])
                       & (wb_csr_we & (wb_csr_num == `CSR_ASID || wb_csr_num == `CSR_TLBEHI) || wb_tlb_op[`TLB_INST_TLBRD])
                       & wb_valid;

assign has_ale          = (ex_load_op[1] | ex_store_op[1]) & (alu_result[0]) | (ex_load_op[2] | ex_store_op[2]) & (|alu_result[1:0]);

assign ex_vaddr         = ex_ex_in ? ex_pc : alu_result;

assign ex_ex            = ex_ex_in | has_ale | (
                          has_tlbr_data_sram | has_pil_data_sram | has_pis_data_sram |
                          has_pme_data_sram | has_ppi_data_sram ) & ((|ex_load_op) || (|ex_store_op));
assign ex_ecode         = ex_ex_in ? ex_ecode_in : 
                          {6{has_ale}} & `ECODE_ALE | {6{has_tlbr_data_sram}} & `ECODE_TLBR
                        | {6{has_pil_data_sram}} & `ECODE_PIL | {6{has_pis_data_sram}} & `ECODE_PIS
                        | {6{has_pme_data_sram}} & `ECODE_PME | {6{has_ppi_data_sram}} & `ECODE_PPI;

//MEM

assign mem_data = data_sram_rdata;
assign mem_data_b = data_sram_rdata[{mem_vaddr[1:0], 3'b0}+:8];
assign mem_data_h = data_sram_rdata[{mem_vaddr[1], 4'b0}+:16];

assign mem_result = {32{mem_load_op[2]}} & mem_data |
                    {32{mem_load_op[0]}} & {{24{mem_data_b[7] & ~mem_load_op[3]}}, mem_data_b} |
                    {32{mem_load_op[1]}} & {{16{mem_data_h[15] & ~mem_load_op[3]}}, mem_data_h};
assign final_result = ex_result_out | mem_result;

assign mem_ex           = mem_ex_in & mem_valid;
assign mem_ecode        = mem_ecode_in;

//WB

assign rf_we            = wb_rf_we & ~wb_ex;
assign rf_waddr         = wb_rf_waddr;
assign rf_wdata         = wb_rf_wdata;

assign csr_we           = wb_csr_we & wb_valid & ~wb_ex & ~ertn_flush;
assign csr_wnum         = wb_csr_num;
assign csr_wmask        = wb_csr_wmask;
assign csr_wvalue       = wb_csr_wvalue;

assign wb_tlb_op_out = wb_tlb_op & {`TLB_INST_TLBRD_MASK | `TLB_INST_TLBWR_MASK | `TLB_INST_TLBFILL_MASK} & {5{wb_valid}};

assign wb_refetch = (wb_tlb_op[`TLB_INST_TLBRD] || wb_tlb_op[`TLB_INST_TLBWR] || wb_tlb_op[`TLB_INST_TLBFILL]
                 || csr_we & (csr_wnum == `CSR_CRMD || csr_wnum == `CSR_ASID)) & wb_valid;

assign ertn_flush       = wb_ertn & wb_valid;

assign wb_ex            = wb_ex_in & wb_valid;
assign wb_ecode         = wb_ecode_in;
assign wb_esubcode      = 9'b0;

assign debug_wb_pc       = wb_pc;
assign debug_wb_rf_we   = {4{rf_we}} & {4{wb_valid}};
assign debug_wb_rf_wnum  = wb_rf_waddr;
assign debug_wb_rf_wdata = wb_rf_wdata;

regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we & wb_valid),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

alu u_alu(
    .clk        (clk),
    .resetn     (resetn & ~wb_ex & ~(id_valid_out)),
    .alu_op     (ex_alu_op    ),
    .alu_src1   (ex_alu_src1  ),
    .alu_src2   (ex_alu_src2  ),
    .alu_result (alu_result),
    .complete   (alu_complete)
    );

csr u_csr(
    .clk(clk),
    .reset(reset),
    .csr_we(csr_we),
    .csr_wnum(csr_wnum),
    .csr_wmask(csr_wmask),
    .csr_wvalue(csr_wvalue),
    .csr_rnum(csr_rnum),
    .csr_rvalue(csr_rvalue),
    .ex_ra(ex_ra),
    .ex_entry(ex_entry),
    .has_int(has_int),
    .ertn_flush(ertn_flush),
    .wb_pc(wb_pc),
    .wb_vaddr(wb_vaddr),
    .wb_ex(wb_ex),
    .wb_ecode(wb_ecode),
    .wb_esubcode(wb_esubcode),
    .tlb_op(ex_tlb_op_out | wb_tlb_op_out),//op==0 if not valid
    .invtlb_op(ex_invtlb_op_out),//op==0 if not valid
    .invtlb_asid(ex_invtlb_asid),
    .invtlb_vppn(ex_invtlb_vppn),
    .inst_sram_vaddr(inst_sram_vaddr),
    .inst_sram_paddr(inst_sram_paddr),
    .has_tlbr_inst_sram(has_tlbr_inst_sram),
    .has_pif_inst_sram(has_pif_inst_sram),
    .has_ppi_inst_sram(has_ppi_inst_sram),
    .data_sram_vaddr(data_sram_vaddr),
    .data_sram_wr(data_sram_wr),
    .data_sram_paddr(data_sram_paddr),
    .has_tlbr_data_sram(has_tlbr_data_sram),
    .has_pil_data_sram(has_pil_data_sram),
    .has_pis_data_sram(has_pis_data_sram),
    .has_pme_data_sram(has_pme_data_sram),
    .has_ppi_data_sram(has_ppi_data_sram)
);

reg     [63:0] stable_cnt;
always @(posedge clk) begin
    if (reset) begin
        stable_cnt <= 64'h0;
    end else begin
        stable_cnt <= stable_cnt + 1'b1;
    end
end

assign tmr_result = stable_cnt[{ex_tmr_op, 5'b0}+:32];

reg [31:0] debug_counter_inst_to;
reg [31:0] debug_counter_inst_from;
always @(posedge clk) begin
    if(~resetn) begin
        debug_counter_inst_to <= 32'b0;
        debug_counter_inst_from <= 32'b0;
    end
    else begin
        if(inst_sram_req & inst_sram_addr_ok) begin
            debug_counter_inst_to <= debug_counter_inst_to + 1;
        end
        if(inst_sram_data_ok) begin
            debug_counter_inst_from <= debug_counter_inst_from + 1;
        end
    end
end

reg [31:0] debug_counter_data_to;
reg [31:0] debug_counter_data_from;
always @(posedge clk) begin
    if(~resetn) begin
        debug_counter_data_to <= 32'b0;
        debug_counter_data_from <= 32'b0;
    end
    else begin
        if(data_sram_req & data_sram_addr_ok) begin
            debug_counter_data_to <= debug_counter_data_to + 1;
        end
        if(data_sram_data_ok) begin
            debug_counter_data_from <= debug_counter_data_from + 1;
        end
    end
end

endmodule
