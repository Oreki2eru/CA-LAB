`include "macro.vh"
module csr(
    input          clk,
    input          reset,
    input          csr_we,
    input   [13:0] csr_wnum,
    input   [31:0] csr_wmask,
    input   [31:0] csr_wvalue,

    input   [13:0] csr_rnum,
    output  [31:0] csr_rvalue,

    output  [31:0] ex_ra,
    output  [31:0] ex_entry,
    output         has_int,
    input          ertn_flush,
    input   [31:0] wb_pc,
    input   [31:0] wb_vaddr,
    input          wb_ex,
    input   [ 5:0] wb_ecode,
    input   [ 8:0] wb_esubcode,

    input   [ 4:0] tlb_op,
    input   [ 4:0] invtlb_op,
    input   [ 9:0] invtlb_asid,
    input   [18:0] invtlb_vppn,

    input   [31:0] inst_sram_vaddr,
    output  [31:0] inst_sram_paddr,
    output         has_tlbr_inst_sram,
    output         has_pif_inst_sram,
    output         has_ppi_inst_sram,


    input   [31:0] data_sram_vaddr,
    input          data_sram_wr,
    output  [31:0] data_sram_paddr,
    output         has_tlbr_data_sram,
    output         has_pil_data_sram,
    output         has_pis_data_sram,
    output         has_pme_data_sram,
    output         has_ppi_data_sram
);
//new: TLB 重填例外(tlbr)、load/store/取指操作页无效例外(pil/pis/pif)、页修改例外(pme)、页特权等级不合规例外(ppi)

//csr regs

//crmd
//TODO: datf datm
wire    [31:0] csr_crmd_rvalue;
reg     [ 1:0] csr_crmd_plv;
reg            csr_crmd_ie;
reg            csr_crmd_da;
reg            csr_crmd_pg;
reg     [ 1:0] csr_crmd_datf;
reg     [ 1:0] csr_crmd_datm;

//prmd
wire    [31:0] csr_prmd_rvalue;
reg     [ 1:0] csr_prmd_pplv;
reg            csr_prmd_pie;

//ecfg
wire    [31:0] csr_ecfg_rvalue;
reg     [12:0] csr_ecfg_lie;

//estat
wire    [31:0] csr_estat_rvalue;
reg     [12:0] csr_estat_is;
reg     [ 5:0] csr_estat_ecode;
reg     [ 8:0] csr_estat_esubcode;

//era
wire    [31:0] csr_era_rvalue;
reg     [31:0] csr_era_pc;

//badv
wire    [31:0] csr_badv_rvalue;
reg     [31:0] csr_badv_vaddr;

//eentry
wire    [31:0] csr_eentry_rvalue;
reg     [25:0] csr_eentry_va;

//save0~3
reg     [31:0] csr_save0_data;
reg     [31:0] csr_save1_data;
reg     [31:0] csr_save2_data;
reg     [31:0] csr_save3_data;

//tid
wire    [31:0] csr_tid_rvalue;
reg     [31:0] csr_tid_tid;

//tcfg
wire    [31:0] csr_tcfg_rvalue;
reg            csr_tcfg_en;
reg            csr_tcfg_periodic;
reg     [29:0] csr_tcfg_initval;

wire    [31:0] tcfg_next_value;

//tval
wire    [31:0] csr_tval_rvalue;

//ticlr
wire    [31:0] csr_ticlr_rvalue;

//TODO
reg     [31:0] timer_cnt;

//tlb
//TLBIDX TLBEHI TLBELO0 TLBELO1 ASID TLBRENTRY
wire    [31:0] csr_tlbidx_rvalue;
reg     [ 3:0] csr_tlbidx_index;//15:0
reg     [ 5:0] csr_tlbidx_ps;
reg            csr_tlbidx_ne;

wire    [31:0] csr_tlbehi_rvalue;
reg     [18:0] csr_tlbehi_vppn;

wire    [31:0] csr_tlbelo0_rvalue;
reg            csr_tlbelo0_v;
reg            csr_tlbelo0_d;
reg     [ 1:0] csr_tlbelo0_plv;
reg     [ 1:0] csr_tlbelo0_mat;
reg            csr_tlbelo0_g;
reg     [19:0] csr_tlbelo0_ppn;

wire    [31:0] csr_tlbelo1_rvalue;
reg            csr_tlbelo1_v;
reg            csr_tlbelo1_d;
reg     [ 1:0] csr_tlbelo1_plv;
reg     [ 1:0] csr_tlbelo1_mat;
reg            csr_tlbelo1_g;
reg     [19:0] csr_tlbelo1_ppn;

wire    [31:0] csr_asid_rvalue;
reg     [ 9:0] csr_asid_asid;
reg     [ 7:0] csr_asid_asidbits;

wire    [31:0] csr_tlbrentry_rvalue;
reg     [25:0] csr_tlbrentry_pa;

//dmw csr
wire    [31:0] csr_dmw0_rvalue;
reg            csr_dmw0_plv0;
reg            csr_dmw0_plv3;
reg     [ 1:0] csr_dmw0_mat;
reg     [ 2:0] csr_dmw0_pseg;
reg     [ 2:0] csr_dmw0_vseg;

wire    [31:0] csr_dmw1_rvalue;
reg            csr_dmw1_plv0;
reg            csr_dmw1_plv3;
reg     [ 1:0] csr_dmw1_mat;
reg     [ 2:0] csr_dmw1_pseg;
reg     [ 2:0] csr_dmw1_vseg;

always @(posedge clk) begin
    if (reset) begin
        csr_crmd_plv <= 2'b0;
        csr_crmd_ie  <= 1'b0;
    end
    else if (wb_ex) begin
        csr_crmd_plv <= 2'b0;
        csr_crmd_ie  <= 1'b0;
    end
    else if (ertn_flush) begin
        csr_crmd_plv <= csr_prmd_pplv;
        csr_crmd_ie  <= csr_prmd_pie;
    end
    else if (csr_we && csr_wnum==`CSR_CRMD) begin
        csr_crmd_plv <= csr_wmask[`CSR_CRMD_PLV] & csr_wvalue[`CSR_CRMD_PLV]
                     | ~csr_wmask[`CSR_CRMD_PLV] & csr_crmd_plv;
        csr_crmd_ie  <= csr_wmask[`CSR_CRMD_IE] & csr_wvalue[`CSR_CRMD_IE]
                     | ~csr_wmask[`CSR_CRMD_IE] & csr_crmd_ie;
    end
end

always @(posedge clk) begin
    if (reset) begin
        csr_crmd_da <= 1'b1;
        csr_crmd_pg <= 1'b0;
        csr_crmd_datf <= 2'b0;
        csr_crmd_datm <= 2'b0;
    end
    else if (csr_we && csr_wnum==`CSR_CRMD) begin
        csr_crmd_da <= csr_wmask[`CSR_CRMD_DA] & csr_wvalue[`CSR_CRMD_DA]
                    | ~csr_wmask[`CSR_CRMD_DA] & csr_crmd_da;
        csr_crmd_pg <= csr_wmask[`CSR_CRMD_PG] & csr_wvalue[`CSR_CRMD_PG]
                    | ~csr_wmask[`CSR_CRMD_PG] & csr_crmd_pg;
        csr_crmd_datf <= csr_wmask[`CSR_CRMD_DATF] & csr_wvalue[`CSR_CRMD_DATF]
                      | ~csr_wmask[`CSR_CRMD_DATF] & csr_crmd_datf;
        csr_crmd_datm <= csr_wmask[`CSR_CRMD_DATM] & csr_wvalue[`CSR_CRMD_DATM]
                      | ~csr_wmask[`CSR_CRMD_DATM] & csr_crmd_datm;
    end
    else if (wb_ex && wb_ecode == `ECODE_TLBR) begin
        csr_crmd_da <= 1'b1;
        csr_crmd_pg <= 1'b0;
    end
    else if (ertn_flush && csr_estat_ecode == `ECODE_TLBR) begin
        csr_crmd_da <= 1'b0;
        csr_crmd_pg <= 1'b1;
    end
end

assign csr_crmd_rvalue = {23'b0, csr_crmd_datm, csr_crmd_datf, csr_crmd_pg, csr_crmd_da, csr_crmd_ie, csr_crmd_plv};//TODO

always @(posedge clk) begin
    if (wb_ex) begin
        csr_prmd_pplv <= csr_crmd_plv;
        csr_prmd_pie  <= csr_crmd_ie;
    end else if (csr_we && csr_wnum==`CSR_PRMD) begin
        csr_prmd_pplv <= csr_wmask[`CSR_PRMD_PPLV] & csr_wvalue[`CSR_PRMD_PPLV]
                      | ~csr_wmask[`CSR_PRMD_PPLV] & csr_prmd_pplv;
        csr_prmd_pie  <= csr_wmask[`CSR_PRMD_PIE] & csr_wvalue[`CSR_PRMD_PIE]
                      | ~csr_wmask[`CSR_PRMD_PIE] & csr_prmd_pie;
    end
end

assign csr_prmd_rvalue = {29'b0, csr_prmd_pie, csr_prmd_pplv};

always @(posedge clk) begin
    if (reset) begin
        csr_ecfg_lie <= 13'b0;
    end else if (csr_we && csr_wnum==`CSR_ECFG) begin
        csr_ecfg_lie <= csr_wmask[`CSR_ECFG_LIE] & 13'h1bff & csr_wvalue[`CSR_ECFG_LIE]
                     | ~csr_wmask[`CSR_ECFG_LIE] & 13'h1bff & csr_ecfg_lie;
    end
end

assign csr_ecfg_rvalue = {19'b0, csr_ecfg_lie};

always @(posedge clk) begin
    if (reset) begin
        csr_estat_is[1:0] <= 2'b0;
    end
    else if (csr_we && csr_wnum==`CSR_ESTAT) begin
        csr_estat_is[1:0] <= csr_wmask[`CSR_ESTAT_IS] & csr_wvalue[`CSR_ESTAT_IS]
                          | ~csr_wmask[`CSR_ESTAT_IS] & csr_estat_is[1:0];
    end

    csr_estat_is[9:2] <= 8'b0;//hw_int_in[7:0];

    csr_estat_is[10] <= 1'b0;

    if (timer_cnt[31:0]==32'b0) begin
        csr_estat_is[11] <= 1'b1;
    end
    else if (csr_we && csr_wnum==`CSR_TICLR && csr_wmask[`CSR_TICLR_CLR]
                        && csr_wvalue[`CSR_TICLR_CLR]) begin
        csr_estat_is[11] <= 1'b0;
    end

    csr_estat_is[12] <= 1'b0;//ipi_int_in;
end

always @(posedge clk) begin
    if (wb_ex) begin
        csr_estat_ecode     <= wb_ecode;
        csr_estat_esubcode  <= wb_esubcode;
    end
end

assign csr_estat_rvalue = {1'b0, csr_estat_esubcode, csr_estat_ecode, 3'b0, csr_estat_is};

always @(posedge clk) begin
    if (wb_ex) begin
        csr_era_pc <= wb_pc;
    end
    else if (csr_we && csr_wnum==`CSR_ERA) begin
        csr_era_pc <= csr_wmask[`CSR_ERA_PC] & csr_wvalue[`CSR_ERA_PC]
                   | ~csr_wmask[`CSR_ERA_PC] & csr_era_pc;
    end
end

assign csr_era_rvalue = csr_era_pc;
//new: TLB 重填例外(tlbr)、load/store/取指操作页无效例外(pil/pis/pif)、页修改例外(pme)、页特权等级不合规例外(ppi)
//TODO:
assign wb_ex_addr_err = wb_ecode==`ECODE_ADE || wb_ecode==`ECODE_ALE
                     || wb_ecode==`ECODE_TLBR || wb_ecode==`ECODE_PIL || wb_ecode==`ECODE_PIS
                     || wb_ecode==`ECODE_PIF || wb_ecode==`ECODE_PME || wb_ecode==`ECODE_PPI;

always @(posedge clk) begin
    if (wb_ex && wb_ex_addr_err) begin
        csr_badv_vaddr <= (wb_ecode==`ECODE_ADE && wb_esubcode==`ESUBCODE_ADEF) ? wb_pc : wb_vaddr;
    end
    else if (csr_we && csr_wnum == `CSR_BADV) begin
        csr_badv_vaddr <= csr_wmask[`CSR_BADV_VADDR] & csr_wvalue[`CSR_BADV_VADDR]
                       | ~csr_wmask[`CSR_BADV_VADDR] & csr_badv_vaddr;
    end
end

assign csr_badv_rvalue = csr_badv_vaddr;

always @(posedge clk) begin
    if (csr_we && csr_wnum==`CSR_EENTRY) begin
        csr_eentry_va <= csr_wmask[`CSR_EENTRY_VA] & csr_wvalue[`CSR_EENTRY_VA]
                      | ~csr_wmask[`CSR_EENTRY_VA] & csr_eentry_va;
    end
end

assign csr_eentry_rvalue = {csr_eentry_va, 6'b0};

always @(posedge clk) begin
    if (csr_we && csr_wnum==`CSR_SAVE0) begin
        csr_save0_data <= csr_wmask[`CSR_SAVE0_DATA] & csr_wvalue[`CSR_SAVE0_DATA]
                       | ~csr_wmask[`CSR_SAVE0_DATA] & csr_save0_data;
    end
    if (csr_we && csr_wnum==`CSR_SAVE1) begin
        csr_save1_data <= csr_wmask[`CSR_SAVE1_DATA] & csr_wvalue[`CSR_SAVE1_DATA]
                       | ~csr_wmask[`CSR_SAVE1_DATA] & csr_save1_data;
    end
    if (csr_we && csr_wnum==`CSR_SAVE2) begin
        csr_save2_data <= csr_wmask[`CSR_SAVE2_DATA] & csr_wvalue[`CSR_SAVE2_DATA]
                       | ~csr_wmask[`CSR_SAVE2_DATA] & csr_save2_data;
    end
    if (csr_we && csr_wnum==`CSR_SAVE3) begin
        csr_save3_data <= csr_wmask[`CSR_SAVE3_DATA] & csr_wvalue[`CSR_SAVE3_DATA]
                       | ~csr_wmask[`CSR_SAVE3_DATA] & csr_save3_data;
    end
end

always @(posedge clk) begin
    if (reset) begin
        csr_tid_tid <= 32'b0;//coreid_in;
    end else if (csr_we && csr_wnum==`CSR_TID) begin
        csr_tid_tid <= csr_wmask[`CSR_TID_TID] & csr_wvalue[`CSR_TID_TID]
                    | ~csr_wmask[`CSR_TID_TID] & csr_tid_tid;
    end
end

assign csr_tid_rvalue = csr_tid_tid;

always @(posedge clk) begin
    if (reset) begin
        csr_tcfg_en <= 1'b0;
    end else if (csr_we && csr_wnum==`CSR_TCFG) begin
        csr_tcfg_en <= csr_wmask[`CSR_TCFG_EN] & csr_wvalue[`CSR_TCFG_EN]
                    | ~csr_wmask[`CSR_TCFG_EN] & csr_tcfg_en;
    end
    
    if (csr_we && csr_wnum==`CSR_TCFG) begin
        csr_tcfg_periodic <= csr_wmask[`CSR_TCFG_PERIODIC] & csr_wvalue[`CSR_TCFG_PERIODIC]
                          | ~csr_wmask[`CSR_TCFG_PERIODIC] & csr_tcfg_periodic;
        csr_tcfg_initval <= csr_wmask[`CSR_TCFG_INITVAL] & csr_wvalue[`CSR_TCFG_INITVAL]
                         | ~csr_wmask[`CSR_TCFG_INITVAL] & csr_tcfg_initval;
    end
end

assign csr_tcfg_rvalue = {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};

assign tcfg_next_value = csr_wmask[31:0] & csr_wvalue[31:0]
                      | ~csr_wmask[31:0] & {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};

always @(posedge clk) begin
    if (reset) begin
        timer_cnt <= 32'hffffffff;
    end else if (csr_we && csr_wnum==`CSR_TCFG && tcfg_next_value[`CSR_TCFG_EN]) begin
        timer_cnt <= {tcfg_next_value[`CSR_TCFG_INITVAL], 2'b0};
    end else if (csr_tcfg_en && timer_cnt!=32'hffffffff) begin
        if (timer_cnt[31:0]==32'b0 && csr_tcfg_periodic) begin
            timer_cnt <= {csr_tcfg_initval, 2'b0};
        end else begin
            timer_cnt <= timer_cnt - 1'b1;
        end
    end
end

assign csr_tval_rvalue = timer_cnt[31:0];

assign csr_ticlr_rvalue = 32'b0;

assign csr_rvalue = {32{csr_rnum == `CSR_CRMD       }} & csr_crmd_rvalue
                  | {32{csr_rnum == `CSR_PRMD       }} & csr_prmd_rvalue
                  | {32{csr_rnum == `CSR_ECFG       }} & csr_ecfg_rvalue
                  | {32{csr_rnum == `CSR_ESTAT      }} & csr_estat_rvalue
                  | {32{csr_rnum == `CSR_ERA        }} & csr_era_rvalue
                  | {32{csr_rnum == `CSR_BADV       }} & csr_badv_rvalue
                  | {32{csr_rnum == `CSR_EENTRY     }} & csr_eentry_rvalue
                  | {32{csr_rnum == `CSR_SAVE0      }} & csr_save0_data
                  | {32{csr_rnum == `CSR_SAVE1      }} & csr_save1_data
                  | {32{csr_rnum == `CSR_SAVE2      }} & csr_save2_data
                  | {32{csr_rnum == `CSR_SAVE3      }} & csr_save3_data
                  | {32{csr_rnum == `CSR_TID        }} & csr_tid_rvalue
                  | {32{csr_rnum == `CSR_TCFG       }} & csr_tcfg_rvalue
                  | {32{csr_rnum == `CSR_TVAL       }} & csr_tval_rvalue
                  | {32{csr_rnum == `CSR_TICLR      }} & csr_ticlr_rvalue
                  | {32{csr_rnum == `CSR_TLBIDX     }} & csr_tlbidx_rvalue
                  | {32{csr_rnum == `CSR_TLBEHI     }} & csr_tlbehi_rvalue
                  | {32{csr_rnum == `CSR_TLBELO0    }} & csr_tlbelo0_rvalue
                  | {32{csr_rnum == `CSR_TLBELO1    }} & csr_tlbelo1_rvalue
                  | {32{csr_rnum == `CSR_ASID       }} & csr_asid_rvalue
                  | {32{csr_rnum == `CSR_TLBRENTRY  }} & csr_tlbrentry_rvalue
                  | {32{csr_rnum == `CSR_DMW0       }} & csr_dmw0_rvalue
                  | {32{csr_rnum == `CSR_DMW1       }} & csr_dmw1_rvalue;

assign ex_entry = (wb_ecode == 6'h3f) ? csr_tlbrentry_rvalue :csr_eentry_rvalue;
assign ex_ra = csr_era_rvalue;
assign has_int = (|(csr_estat_is & csr_ecfg_lie)) & csr_crmd_ie;

// search port 0 (for fetch)
wire [18:0]               s0_vppn;
wire                      s0_va_bit12;
wire [ 9:0]               s0_asid;
wire                      s0_found;
wire [ 3:0]               s0_index;
wire [19:0]               s0_ppn;
wire [ 5:0]               s0_ps;
wire [ 1:0]               s0_plv;
wire [ 1:0]               s0_mat;
wire                      s0_d;
wire                      s0_v;
// search port 1 (for load/store)
wire [18:0]               s1_vppn;
wire                      s1_va_bit12;
wire [ 9:0]               s1_asid;
wire                      s1_found;
wire [ 3:0]               s1_index;
wire [19:0]               s1_ppn;
wire [ 5:0]               s1_ps;
wire [ 1:0]               s1_plv;
wire [ 1:0]               s1_mat;
wire                      s1_d;
wire                      s1_v;
// invtlb opcode
wire                      invtlb_valid;
//wire [ 4:0]               invtlb_op;
// write port
wire                      we; //w(rite) e(nable)
wire [ 3:0]               w_index;
wire                      w_e;
wire [18:0]               w_vppn;
wire [ 5:0]               w_ps;
wire [ 9:0]               w_asid;
wire                      w_g;
wire [19:0]               w_ppn0;
wire [ 1:0]               w_plv0;
wire [ 1:0]               w_mat0;
wire                      w_d0;
wire                      w_v0;
wire [19:0]               w_ppn1;
wire [ 1:0]               w_plv1;
wire [ 1:0]               w_mat1;
wire                      w_d1;
wire                      w_v1;
// read port
wire [ 3:0]               r_index;
wire                      r_e;
wire [18:0]               r_vppn;
wire [ 5:0]               r_ps;
wire [ 9:0]               r_asid;
wire                      r_g;
wire [19:0]               r_ppn0;
wire [ 1:0]               r_plv0;
wire [ 1:0]               r_mat0;
wire                      r_d0;
wire                      r_v0;
wire [19:0]               r_ppn1;
wire [ 1:0]               r_plv1;
wire [ 1:0]               r_mat1;
wire                      r_d1;
wire                      r_v1;

tlb tlb_u(
clk,
// search port 0 (for fetch)
s0_vppn,
s0_va_bit12,
s0_asid,
s0_found,
s0_index,
s0_ppn,
s0_ps,
s0_plv,
s0_mat,
s0_d,
s0_v,
// search port 1 (for load/store)
s1_vppn,
s1_va_bit12,
s1_asid,
s1_found,
s1_index,
s1_ppn,
s1_ps,
s1_plv,
s1_mat,
s1_d,
s1_v,
// invtlb opcode
invtlb_valid,
invtlb_op,
// write port
we, //w(rite) e(nable)
w_index,
w_e,
w_vppn,
w_ps,
w_asid,
w_g,
w_ppn0,
w_plv0,
w_mat0,
w_d0,
w_v0,
w_ppn1,
w_plv1,
w_mat1,
w_d1,
w_v1,
// read port
r_index,
r_e,
r_vppn,
r_ps,
r_asid,
r_g,
r_ppn0,
r_plv0,
r_mat0,
r_d0,
r_v0,
r_ppn1,
r_plv1,
r_mat1,
r_d1,
r_v1
);

assign we = tlb_op[`TLB_INST_TLBWR] || tlb_op[`TLB_INST_TLBFILL];
assign invtlb_valid = tlb_op[`TLB_INST_INVTLB];

// generate random index for tlbfill
reg  [3:0] rand_index;

assign r_index = csr_tlbidx_index;

always @(posedge clk) begin
    rand_index <= $random;
end

assign w_index = {4{tlb_op[`TLB_INST_TLBWR]}} & csr_tlbidx_index
               | {4{tlb_op[`TLB_INST_TLBFILL]}} & rand_index;
assign w_e = ~csr_tlbidx_ne || csr_estat_ecode == 6'h3f;//do not need && due to we
assign w_ps = csr_tlbidx_ps;

//tlb
always @(posedge clk) begin
    if (reset) begin
        csr_tlbidx_index <= 4'b0;
        csr_tlbidx_ps <= 6'b0;
        csr_tlbidx_ne <= 1'b0;
    end
    else if (csr_we && csr_wnum == `CSR_TLBIDX) begin
        csr_tlbidx_index <= csr_wmask[`CSR_TLBIDX_INDEX] & csr_wvalue[`CSR_TLBIDX_INDEX]
                         | ~csr_wmask[`CSR_TLBIDX_INDEX] & csr_tlbidx_index;
        csr_tlbidx_ps    <= csr_wmask[`CSR_TLBIDX_PS] & csr_wvalue[`CSR_TLBIDX_PS]
                         | ~csr_wmask[`CSR_TLBIDX_PS] & csr_tlbidx_ps;
        csr_tlbidx_ne    <= csr_wmask[`CSR_TLBIDX_NE] & csr_wvalue[`CSR_TLBIDX_NE]
                         | ~csr_wmask[`CSR_TLBIDX_NE] & csr_tlbidx_ne;
    end
    else if (tlb_op[`TLB_INST_TLBSRCH]) begin
        csr_tlbidx_index <= s1_found ? s1_index : csr_tlbidx_index;//if hit, index
        csr_tlbidx_ne <= ~s1_found;//if hit, 0
    end
    else if (tlb_op[`TLB_INST_TLBRD]) begin
        csr_tlbidx_ps <= r_e ? r_ps : 6'b0;//ps
        csr_tlbidx_ne <= ~r_e;//~e
    end
end
//tlbidx对于hit的判断似乎应当是对于数据内存访存的端口而言的
assign csr_tlbidx_rvalue = {csr_tlbidx_ne, 1'b0, csr_tlbidx_ps, 8'b0, 12'b0, csr_tlbidx_index};

//assign s1_vppn = {19{tlb_op[`TLB_INST_TLBSRCH]}} & csr_tlbehi_vppn
//               | {19{tlb_op[`TLB_INST_INVTLB]}} & invtlb_vppn;
assign w_vppn = csr_tlbehi_vppn;

always @(posedge clk) begin
    if (reset) begin
        csr_tlbehi_vppn <= 19'b0;
    end
    else if (csr_we && csr_wnum == `CSR_TLBEHI) begin
        csr_tlbehi_vppn <= csr_wmask[`CSR_TLBEHI_VPPN] & csr_wvalue[`CSR_TLBEHI_VPPN]
                        | ~csr_wmask[`CSR_TLBEHI_VPPN] & csr_tlbehi_vppn;
    end
    else if (wb_ex && (wb_ecode == `ECODE_TLBR || wb_ecode == `ECODE_PIL || wb_ecode == `ECODE_PIS 
                    || wb_ecode == `ECODE_PIF || wb_ecode == `ECODE_PME || wb_ecode == `ECODE_PPI)) begin
//new: TLB 重填例外(tlbr)、load/store/取指操作页无效例外(pil/pis/pif)、页修改例外(pme)、页特权等级不合规例外(ppi)
        csr_tlbehi_vppn <= wb_vaddr[31:13];
    end
    else if (tlb_op[`TLB_INST_TLBRD]) begin
        csr_tlbehi_vppn <= r_e ? r_vppn : 19'b0;//vppn
    end
end

assign csr_tlbehi_rvalue = {csr_tlbehi_vppn, 13'b0};

assign w_g = csr_tlbelo0_g && csr_tlbelo1_g;
assign w_ppn0 = csr_tlbelo0_ppn;
assign w_plv0 = csr_tlbelo0_plv;
assign w_mat0 = csr_tlbelo0_plv;
assign w_d0 = csr_tlbelo0_d;
assign w_v0 = csr_tlbelo0_v;
assign w_ppn1 = csr_tlbelo1_ppn;
assign w_plv1 = csr_tlbelo1_plv;
assign w_mat1 = csr_tlbelo1_mat;
assign w_d1 = csr_tlbelo1_d;
assign w_v1 = csr_tlbelo1_v;

always @(posedge clk) begin
    if (reset) begin
        csr_tlbelo0_v   <= 1'b0;
        csr_tlbelo0_d   <= 1'b0;
        csr_tlbelo0_plv <= 2'b0;
        csr_tlbelo0_mat <= 2'b0;
        csr_tlbelo0_g   <= 1'b0;
        csr_tlbelo0_ppn <= 20'b0;
    end
    else if (csr_we && csr_wnum == `CSR_TLBELO0) begin
        csr_tlbelo0_v   <= csr_wmask[`CSR_TLBELO0_V] & csr_wvalue[`CSR_TLBELO0_V]
                        | ~csr_wmask[`CSR_TLBELO0_V] & csr_tlbelo0_v;
        csr_tlbelo0_d   <= csr_wmask[`CSR_TLBELO0_D] & csr_wvalue[`CSR_TLBELO0_D]
                        | ~csr_wmask[`CSR_TLBELO0_D] & csr_tlbelo0_d;
        csr_tlbelo0_plv <= csr_wmask[`CSR_TLBELO0_PLV] & csr_wvalue[`CSR_TLBELO0_PLV]
                        | ~csr_wmask[`CSR_TLBELO0_PLV] & csr_tlbelo0_plv;
        csr_tlbelo0_mat <= csr_wmask[`CSR_TLBELO0_MAT] & csr_wvalue[`CSR_TLBELO0_MAT]
                        | ~csr_wmask[`CSR_TLBELO0_MAT] & csr_tlbelo0_mat;
        csr_tlbelo0_g   <= csr_wmask[`CSR_TLBELO0_G] & csr_wvalue[`CSR_TLBELO0_G]
                        | ~csr_wmask[`CSR_TLBELO0_G] & csr_tlbelo0_g;
        csr_tlbelo0_ppn <= csr_wmask[`CSR_TLBELO0_PPN] & csr_wvalue[`CSR_TLBELO0_PPN]
                        | ~csr_wmask[`CSR_TLBELO0_PPN] & csr_tlbelo0_ppn;
    end
    else if (tlb_op[`TLB_INST_TLBRD]) begin
        csr_tlbelo0_v   <= r_e ? r_v0 : 1'b0;
        csr_tlbelo0_d   <= r_e ? r_d0 : 1'b0;
        csr_tlbelo0_plv <= r_e ? r_plv0 : 2'b0;
        csr_tlbelo0_mat <= r_e ? r_mat0 : 2'b0;
        csr_tlbelo0_g   <= r_e ? r_g : 1'b0;
        csr_tlbelo0_ppn <= r_e ? r_ppn0 : 20'b0;
    end
end

always @(posedge clk) begin
    if (reset) begin
        csr_tlbelo1_v   <= 1'b0;
        csr_tlbelo1_d   <= 1'b0;
        csr_tlbelo1_plv <= 2'b0;
        csr_tlbelo1_mat <= 2'b0;
        csr_tlbelo1_g   <= 1'b0;
        csr_tlbelo1_ppn <= 20'b0;
    end
    else if (csr_we && csr_wnum == `CSR_TLBELO1) begin
        csr_tlbelo1_v   <= csr_wmask[`CSR_TLBELO1_V] & csr_wvalue[`CSR_TLBELO1_V]
                        | ~csr_wmask[`CSR_TLBELO1_V] & csr_tlbelo1_v;
        csr_tlbelo1_d   <= csr_wmask[`CSR_TLBELO1_D] & csr_wvalue[`CSR_TLBELO1_D]
                        | ~csr_wmask[`CSR_TLBELO1_D] & csr_tlbelo1_d;
        csr_tlbelo1_plv <= csr_wmask[`CSR_TLBELO1_PLV] & csr_wvalue[`CSR_TLBELO1_PLV]
                        | ~csr_wmask[`CSR_TLBELO1_PLV] & csr_tlbelo1_plv;
        csr_tlbelo1_mat <= csr_wmask[`CSR_TLBELO1_MAT] & csr_wvalue[`CSR_TLBELO1_MAT]
                        | ~csr_wmask[`CSR_TLBELO1_MAT] & csr_tlbelo1_mat;
        csr_tlbelo1_g   <= csr_wmask[`CSR_TLBELO1_G] & csr_wvalue[`CSR_TLBELO1_G]
                        | ~csr_wmask[`CSR_TLBELO1_G] & csr_tlbelo1_g;
        csr_tlbelo1_ppn <= csr_wmask[`CSR_TLBELO1_PPN] & csr_wvalue[`CSR_TLBELO1_PPN]
                        | ~csr_wmask[`CSR_TLBELO1_PPN] & csr_tlbelo1_ppn;
    end
    else if (tlb_op[`TLB_INST_TLBRD]) begin
        csr_tlbelo1_v   <= r_e ? r_v1 : 1'b0;
        csr_tlbelo1_d   <= r_e ? r_d1 : 1'b0;
        csr_tlbelo1_plv <= r_e ? r_plv1 : 2'b0;
        csr_tlbelo1_mat <= r_e ? r_mat1 : 2'b0;
        csr_tlbelo1_g   <= r_e ? r_g : 1'b0;
        csr_tlbelo1_ppn <= r_e ? r_ppn1 : 20'b0;
    end
end

assign csr_tlbelo0_rvalue = {4'b0, csr_tlbelo0_ppn, 1'b0, csr_tlbelo0_g, csr_tlbelo0_mat, csr_tlbelo0_plv, csr_tlbelo0_d, csr_tlbelo0_v};
assign csr_tlbelo1_rvalue = {4'b0, csr_tlbelo1_ppn, 1'b0, csr_tlbelo1_g, csr_tlbelo1_mat, csr_tlbelo1_plv, csr_tlbelo1_d, csr_tlbelo1_v};

//assign s0_asid = csr_asid_asid;
//assign s1_asid = {10{tlb_op[`TLB_INST_TLBSRCH]}} & csr_asid_asid
//               | {10{tlb_op[`TLB_INST_INVTLB]}} & invtlb_asid;
assign w_asid = csr_asid_asid;

always @(posedge clk) begin
    if (reset) begin
        csr_asid_asid <= 10'b0;
        csr_asid_asidbits <= 8'd10;
    end
    else if (csr_we && csr_wnum == `CSR_ASID) begin
        csr_asid_asid <= csr_wmask[`CSR_ASID_ASID] & csr_wvalue[`CSR_ASID_ASID]
                      | ~csr_wmask[`CSR_ASID_ASID] & csr_asid_asid; 
    end
    else if (tlb_op[`TLB_INST_TLBRD]) begin
        csr_asid_asid <= r_e ? r_asid : 10'b0;//
    end
end

assign csr_asid_rvalue = {8'b0, csr_asid_asidbits, 6'b0, csr_asid_asid};

always @(posedge clk) begin
    if (reset) begin
        csr_tlbrentry_pa <= 26'b0;
    end
    else if (csr_we && csr_wnum == `CSR_TLBRENTRY) begin
        csr_tlbrentry_pa <= csr_wmask[`CSR_TLBRENTRY_PA] & csr_wvalue[`CSR_TLBRENTRY_PA]
                         | ~csr_wmask[`CSR_TLBRENTRY_PA] & csr_tlbrentry_pa;
    end
end

assign csr_tlbrentry_rvalue = {csr_tlbrentry_pa, 6'b0};

always @(posedge clk) begin
    if (reset) begin
        csr_dmw0_plv0 <= 1'b0;
        csr_dmw0_plv3 <= 1'b0;
        csr_dmw0_mat <= 2'b0;
        csr_dmw0_pseg <= 3'b0;
        csr_dmw0_vseg <= 3'b0;
    end
    else if (csr_we && csr_wnum == `CSR_DMW0) begin
        csr_dmw0_plv0 <= csr_wmask[`CSR_DMW0_PLV0] & csr_wvalue[`CSR_DMW0_PLV0]
                      | ~csr_wmask[`CSR_DMW0_PLV0] & csr_dmw0_plv0;
        csr_dmw0_plv3 <= csr_wmask[`CSR_DMW0_PLV3] & csr_wvalue[`CSR_DMW0_PLV3]
                      | ~csr_wmask[`CSR_DMW0_PLV3] & csr_dmw0_plv3;
        csr_dmw0_mat <= csr_wmask[`CSR_DMW0_MAT] & csr_wvalue[`CSR_DMW0_MAT]
                     | ~csr_wmask[`CSR_DMW0_MAT] & csr_dmw0_mat;
        csr_dmw0_pseg <= csr_wmask[`CSR_DMW0_PSEG] & csr_wvalue[`CSR_DMW0_PSEG]
                      | ~csr_wmask[`CSR_DMW0_PSEG] & csr_dmw0_pseg;
        csr_dmw0_vseg <= csr_wmask[`CSR_DMW0_VSEG] & csr_wvalue[`CSR_DMW0_VSEG]
                      | ~csr_wmask[`CSR_DMW0_VSEG] & csr_dmw0_vseg;
    end
end

assign csr_dmw0_rvalue = {csr_dmw0_vseg, 1'b0, csr_dmw0_pseg, 19'b0, csr_dmw0_mat, csr_dmw0_plv3, 2'b0, csr_dmw0_plv0};

always @(posedge clk) begin
    if (reset) begin
        csr_dmw1_plv0 <= 1'b0;
        csr_dmw1_plv3 <= 1'b0;
        csr_dmw1_mat <= 2'b0;
        csr_dmw1_pseg <= 3'b0;
        csr_dmw1_vseg <= 3'b0;
    end
    else if (csr_we && csr_wnum == `CSR_DMW1) begin
        csr_dmw1_plv0 <= csr_wmask[`CSR_DMW1_PLV0] & csr_wvalue[`CSR_DMW1_PLV0]
                      | ~csr_wmask[`CSR_DMW1_PLV0] & csr_dmw1_plv0;
        csr_dmw1_plv3 <= csr_wmask[`CSR_DMW1_PLV3] & csr_wvalue[`CSR_DMW1_PLV3]
                      | ~csr_wmask[`CSR_DMW1_PLV3] & csr_dmw1_plv3;
        csr_dmw1_mat <= csr_wmask[`CSR_DMW1_MAT] & csr_wvalue[`CSR_DMW1_MAT]
                     | ~csr_wmask[`CSR_DMW1_MAT] & csr_dmw1_mat;
        csr_dmw1_pseg <= csr_wmask[`CSR_DMW1_PSEG] & csr_wvalue[`CSR_DMW1_PSEG]
                      | ~csr_wmask[`CSR_DMW1_PSEG] & csr_dmw1_pseg;
        csr_dmw1_vseg <= csr_wmask[`CSR_DMW1_VSEG] & csr_wvalue[`CSR_DMW1_VSEG]
                      | ~csr_wmask[`CSR_DMW1_VSEG] & csr_dmw1_vseg;
    end
end

assign csr_dmw1_rvalue = {csr_dmw1_vseg, 1'b0, csr_dmw1_pseg, 19'b0, csr_dmw1_mat, csr_dmw1_plv3, 2'b0, csr_dmw1_plv0};

assign is_inst_sram_hit_dmw0 = inst_sram_vaddr[31:29] == csr_dmw0_vseg
 && (csr_crmd_plv == 2'b0 && csr_dmw0_plv0 || csr_crmd_plv == 2'b11 && csr_dmw0_plv3);
assign is_inst_sram_hit_dmw1 = inst_sram_vaddr[31:29] == csr_dmw1_vseg
 && (csr_crmd_plv == 2'b0 && csr_dmw1_plv0 || csr_crmd_plv == 2'b11 && csr_dmw1_plv3);
assign is_inst_sram_da = csr_crmd_da && ~csr_crmd_pg;
assign is_inst_sram_dm0 = ~csr_crmd_da && csr_crmd_pg && is_inst_sram_hit_dmw0;
assign is_inst_sram_dm1 = ~csr_crmd_da && csr_crmd_pg && is_inst_sram_hit_dmw1;
assign is_inst_sram_pm = ~csr_crmd_da && csr_crmd_pg && ~is_inst_sram_hit_dmw0 && ~is_inst_sram_hit_dmw1;
wire [31:0] inst_sram_paddr_da;
wire [31:0] inst_sram_paddr_dm0;
wire [31:0] inst_sram_paddr_dm1;
wire [31:0] inst_sram_paddr_pm;
assign inst_sram_paddr_da = inst_sram_vaddr;
assign inst_sram_paddr_dm0 = {csr_dmw0_pseg, inst_sram_vaddr[28:0]};
assign inst_sram_paddr_dm1 = {csr_dmw1_pseg, inst_sram_vaddr[28:0]};
assign inst_sram_paddr_pm = {s0_ppn[19:10], s0_ps == 21 ? inst_sram_vaddr[21:12] : s0_ppn[9:0], inst_sram_vaddr[11:0]};

assign s0_vppn = inst_sram_vaddr[31:13];//??vppn should be 18:0
assign s0_va_bit12 = inst_sram_vaddr[12];
assign s0_asid = csr_asid_asid;

//new: TLB 重填例外(tlbr)、load/store/取指操作页无效例外(pil/pis/pif)、页修改例外(pme)、页特权等级不合规例外(ppi)
assign has_tlbr_inst_sram = is_inst_sram_pm && ~s0_found;
assign has_pif_inst_sram = is_inst_sram_pm && s0_found && ~s0_v;
assign has_ppi_inst_sram = is_inst_sram_pm && s0_found && s0_v && (csr_crmd_plv > s0_plv);

assign inst_sram_paddr = {32{is_inst_sram_da}} & inst_sram_paddr_da | {32{is_inst_sram_dm0}} & inst_sram_paddr_dm0
                       | {32{is_inst_sram_dm1}} & inst_sram_paddr_dm1 | {32{is_inst_sram_pm}} & inst_sram_paddr_pm;

assign is_data_sram_hit_dmw0 = data_sram_vaddr[31:29] == csr_dmw0_vseg
 && (csr_crmd_plv == 2'b0 && csr_dmw0_plv0 || csr_crmd_plv == 2'b11 && csr_dmw0_plv3);
assign is_data_sram_hit_dmw1 = data_sram_vaddr[31:29] == csr_dmw1_vseg
 && (csr_crmd_plv == 2'b0 && csr_dmw1_plv0 || csr_crmd_plv == 2'b11 && csr_dmw1_plv3);
assign is_data_sram_da = csr_crmd_da && ~csr_crmd_pg;
assign is_data_sram_dm0 = ~csr_crmd_da && csr_crmd_pg && is_data_sram_hit_dmw0;
assign is_data_sram_dm1 = ~csr_crmd_da && csr_crmd_pg && is_data_sram_hit_dmw1;
assign is_data_sram_pm = ~csr_crmd_da && csr_crmd_pg && ~is_data_sram_hit_dmw0 && ~is_data_sram_hit_dmw1;
wire [31:0] data_sram_paddr_da;
wire [31:0] data_sram_paddr_dm0;
wire [31:0] data_sram_paddr_dm1;
wire [31:0] data_sram_paddr_pm;
assign data_sram_paddr_da = data_sram_vaddr;
assign data_sram_paddr_dm0 = {csr_dmw0_pseg, data_sram_vaddr[28:0]};
assign data_sram_paddr_dm1 = {csr_dmw1_pseg, data_sram_vaddr[28:0]};
assign data_sram_paddr_pm = {s1_ppn[19:10], s1_ps == 21 ? data_sram_vaddr[21:12] : s1_ppn[9:0], data_sram_vaddr[11:0]};

assign s1_vppn = {19{tlb_op[`TLB_INST_TLBSRCH]}} & csr_tlbehi_vppn | {19{tlb_op[`TLB_INST_INVTLB]}} & invtlb_vppn
               | {19{~tlb_op[`TLB_INST_TLBSRCH] & ~tlb_op[`TLB_INST_INVTLB]}} & data_sram_vaddr[31:13];
assign s1_va_bit12 = ~tlb_op[`TLB_INST_TLBSRCH] & ~tlb_op[`TLB_INST_INVTLB] & & data_sram_vaddr[12];
assign s1_asid = {10{tlb_op[`TLB_INST_TLBSRCH]}} & csr_asid_asid | {10{tlb_op[`TLB_INST_INVTLB]}} & invtlb_asid
               | {10{~tlb_op[`TLB_INST_TLBSRCH] & ~tlb_op[`TLB_INST_INVTLB]}} & csr_asid_asid;

assign has_tlbr_data_sram = is_data_sram_pm && ~s1_found;
assign has_pil_data_sram = is_data_sram_pm && s1_found && ~s1_v && ~data_sram_wr;
assign has_pis_data_sram = is_data_sram_pm && s1_found && ~s1_v && data_sram_wr;
assign has_pme_data_sram = is_data_sram_pm && s1_found && s1_v && ~(csr_crmd_plv > s1_plv) && ~s1_d && data_sram_wr;
assign has_ppi_data_sram = is_data_sram_pm && s1_found && s1_v && (csr_crmd_plv > s1_plv);

assign data_sram_paddr = {32{is_data_sram_da}} & data_sram_paddr_da | {32{is_data_sram_dm0}} & data_sram_paddr_dm0
                       | {32{is_data_sram_dm1}} & data_sram_paddr_dm1 | {32{is_data_sram_pm}} & data_sram_paddr_pm;

endmodule