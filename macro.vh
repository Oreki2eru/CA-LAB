`define BR_BUS_LEN 34
`define IF_to_ID_LEN 112
`define ID_to_EXE_LEN 295
`define RF_BUS_LEN 38
`define EXE_to_MEM_LEN 238
`define MEM_to_WB_LEN 199
`define EXE_RF_LEN 55
`define MEM_RF_LEN 54
`define WB_RF_LEN  37
`define DEST_LEN 5

`define ST_W 7
`define ST_B 6
`define ST_H 5
`define LD_W 4
`define LD_B 3
`define LD_H 2
`define LD_BU 1
`define LD_HU 0

`define CSR_CRMD        14'h0
`define CSR_PRMD        14'h1
`define CSR_EUEN        14'h2
`define CSR_ECFG        14'h4
`define CSR_ESTAT       14'h5
`define CSR_ERA         14'h6
`define CSR_BADV        14'h7
`define CSR_EENTRY      14'hc
`define CSR_TLBIDX      14'h10
`define CSR_TLBEHI      14'h11
`define CSR_TLBELO0     14'h12
`define CSR_TLBELO1     14'h13
`define CSR_ASID        14'h18
`define CSR_PGDL        14'h19
`define CSR_PGDH        14'h1a
`define CSR_PGD         14'h1b
`define CSR_CPUID       14'h20
`define CSR_SAVE0       14'h30
`define CSR_SAVE1       14'h31
`define CSR_SAVE2       14'h32
`define CSR_SAVE3       14'h33
`define CSR_TID         14'h40
`define CSR_TCFG        14'h41
`define CSR_TVAL        14'h42
`define CSR_TICLR       14'h44
`define CSR_LLBCTL      14'h60
`define CSR_TLBRENTRY   14'h88
`define CSR_CTAG        14'h98
`define CSR_DMW0        14'h180
`define CSR_DMW1        14'h181

`define ECODE_INT   6'h0
`define ECODE_PIL   6'h1
`define ECODE_PIS   6'h2
`define ECODE_PIF   6'h3
`define ECODE_PME   6'h4
`define ECODE_PPI   6'h7
`define ECODE_ADE   6'h8
`define ECODE_ALE   6'h9
`define ECODE_SYS   6'hb
`define ECODE_BRK   6'hc
`define ECODE_INE   6'hd
`define ECODE_IPE   6'he
`define ECODE_FPD   6'hf
`define ECODE_FPE   6'h12
`define ECODE_TLBR  6'h3f

`define ESUBCODE_ADEF 8'b0
`define ESUBCODE_ADEM 8'b1

`define CSR_CRMD_PLV    1:0
`define CSR_CRMD_IE     2
`define CSR_CRMD_DA     3
`define CSR_CRMD_PG     4
`define CSR_CRMD_DATF   6:5
`define CSR_CRMD_DATM   8:7
//`define CSR_CRMD_0      31:9

`define CSR_PRMD_PPLV   1:0
`define CSR_PRMD_PIE    2
//`define CSR_PRMD_0      31:3

`define CSR_ESTAT_IS        12:0
//`define CSR_ESTAT_0         15:13
`define CSR_ESTAT_ECODE     21:16
`define CSR_ESTAT_ESUBCODE  30:22
//`define CSR_ESTAT_0         31

`define CSR_ERA_PC      31:0

//`define CSR_EENTRY_0    5:0
`define CSR_EENTRY_VA   31:6

`define CSR_SAVE0_DATA  31:0

`define CSR_SAVE1_DATA  31:0

`define CSR_SAVE2_DATA  31:0

`define CSR_SAVE3_DATA  31:0

`define CSR_ECFG_LIE    12:0
//`define CSR_ECFG_0      31:13

`define CSR_BADV_VADDR  31:0

`define CSR_TID_TID     31:0

`define CSR_TCFG_EN         0
`define CSR_TCFG_PERIODIC   1
`define CSR_TCFG_INITVAL    31:2

`define CSR_TVAL_TIMEVAL    31:0

`define CSR_TICLR_CLR   0
//`define CSR_TICLR_0     31:1

// for tlb
`define CSR_TLBIDX_INDEX    3:0
// `define CSR_TLBIDX_0        23:4
`define CSR_TLBIDX_PS       29:24
// `define CSR_TLBIDX_0        30
`define CSR_TLBIDX_NE       31

// `define CSR_TLBEHI_0        12:0
`define CSR_TLBEHI_VPPN     31:13

`define CSR_TLBELO0_V       0
`define CSR_TLBELO0_D       1
`define CSR_TLBELO0_PLV     3:2
`define CSR_TLBELO0_MAT     5:4
`define CSR_TLBELO0_G       6
// `define CSR_TLBELO0_0       7
`define CSR_TLBELO0_PPN     27:8
// `define CSR_TLBELO0_0       31:28

`define CSR_TLBELO1_V       0
`define CSR_TLBELO1_D       1
`define CSR_TLBELO1_PLV     3:2
`define CSR_TLBELO1_MAT     5:4
`define CSR_TLBELO1_G       6
// `define CSR_TLBELO1_0       7
`define CSR_TLBELO1_PPN     27:8
// `define CSR_TLBELO1_0       31:28

`define CSR_ASID_ASID       9:0
// `define CSR_ASID_0          15:10
`define CSR_ASID_ASIDBITS   23:16
// `define CSR_ASID_0          31:24

// `define CSR_TLBRENTRY_0     5:0
`define CSR_TLBRENTRY_PA    31:6

`define TLB_INST_TLBSRCH    0
`define TLB_INST_TLBRD      1
`define TLB_INST_TLBWR      2
`define TLB_INST_TLBFILL    3
`define TLB_INST_INVTLB     4

`define TLB_INST_TLBSRCH_MASK   5'b00001
`define TLB_INST_TLBRD_MASK     5'b00010
`define TLB_INST_TLBWR_MASK     5'b00100
`define TLB_INST_TLBFILL_MASK   5'b01000
`define TLB_INST_INVTLB_MASK    5'b10000

`define CSR_DMW0_PLV0   0
// `define CSR_DMW0_0      2:1
`define CSR_DMW0_PLV3   3
`define CSR_DMW0_MAT    5:4
// `define CSR_DMW0_0      24:6
`define CSR_DMW0_PSEG   27:25
// `define CSR_DMW0_0      28
`define CSR_DMW0_VSEG   31:29

`define CSR_DMW1_PLV0   0
// `define CSR_DMW1_0      2:1
`define CSR_DMW1_PLV3   3
`define CSR_DMW1_MAT    5:4
// `define CSR_DMW1_0      24:6
`define CSR_DMW1_PSEG   27:25
// `define CSR_DMW1_0      28
`define CSR_DMW1_VSEG   31:29