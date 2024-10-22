module tlb
#(
parameter TLBNUM = 16
)
(
input  wire                      clk,
// search port 0 (for fetch)
input  wire [18:0]               s0_vppn,
input  wire                      s0_va_bit12,
input  wire [ 9:0]               s0_asid,
output wire                      s0_found,
output wire [$clog2(TLBNUM)-1:0] s0_index,
output wire [19:0]               s0_ppn,
output wire [ 5:0]               s0_ps,
output wire [ 1:0]               s0_plv,
output wire [ 1:0]               s0_mat,
output wire                      s0_d,
output wire                      s0_v,
// search port 1 (for load/store)
input  wire [18:0]               s1_vppn,
input  wire                      s1_va_bit12,
input  wire [ 9:0]               s1_asid,
output wire                      s1_found,
output wire [$clog2(TLBNUM)-1:0] s1_index,
output wire [19:0]               s1_ppn,
output wire [ 5:0]               s1_ps,
output wire [ 1:0]               s1_plv,
output wire [ 1:0]               s1_mat,
output wire                      s1_d,
output wire                      s1_v,
// invtlb opcode
input  wire                      invtlb_valid,
input  wire [ 4:0]               invtlb_op,
// write port
input  wire                      we, //w(rite) e(nable)
input  wire [$clog2(TLBNUM)-1:0] w_index,
input  wire                      w_e,
input  wire [18:0]               w_vppn,
input  wire [ 5:0]               w_ps,
input  wire [ 9:0]               w_asid,
input  wire                      w_g,
input  wire [19:0]               w_ppn0,
input  wire [ 1:0]               w_plv0,
input  wire [ 1:0]               w_mat0,
input  wire                      w_d0,
input  wire                      w_v0,
input  wire [19:0]               w_ppn1,
input  wire [ 1:0]               w_plv1,
input  wire [ 1:0]               w_mat1,
input  wire                      w_d1,
input  wire                      w_v1,
// read port
input  wire [$clog2(TLBNUM)-1:0] r_index,
output wire                      r_e,
output wire [18:0]               r_vppn,
output wire [ 5:0]               r_ps,
output wire [ 9:0]               r_asid,
output wire                      r_g,
output wire [19:0]               r_ppn0,
output wire [ 1:0]               r_plv0,
output wire [ 1:0]               r_mat0,
output wire                      r_d0,
output wire                      r_v0,
output wire [19:0]               r_ppn1,
output wire [ 1:0]               r_plv1,
output wire [ 1:0]               r_mat1,
output wire                      r_d1,
output wire                      r_v1
);
reg  [TLBNUM-1:0] tlb_e;
reg  [TLBNUM-1:0] tlb_ps4MB; //pagesize 1:4MB, 0:4KB
reg  [      18:0] tlb_vppn [TLBNUM-1:0];
reg  [       9:0] tlb_asid [TLBNUM-1:0];
reg               tlb_g    [TLBNUM-1:0];
reg  [      19:0] tlb_ppn0 [TLBNUM-1:0];
reg  [       1:0] tlb_plv0 [TLBNUM-1:0];
reg  [       1:0] tlb_mat0 [TLBNUM-1:0];
reg               tlb_d0   [TLBNUM-1:0];
reg               tlb_v0   [TLBNUM-1:0];
reg  [      19:0] tlb_ppn1 [TLBNUM-1:0];
reg  [       1:0] tlb_plv1 [TLBNUM-1:0];
reg  [       1:0] tlb_mat1 [TLBNUM-1:0];
reg               tlb_d1   [TLBNUM-1:0];
reg               tlb_v1   [TLBNUM-1:0];

wire [TLBNUM-1:0] match0;
wire [TLBNUM-1:0] match1;
wire [TLBNUM-1:0] cond1;
wire [TLBNUM-1:0] cond2;
wire [TLBNUM-1:0] cond3;
wire [TLBNUM-1:0] cond4;
wire [TLBNUM-1:0] invtlb_mask;

wire s0_sel;
wire s1_sel;

genvar i;
generate
    for(i = 0; i < TLBNUM; i = i + 1) begin
        assign match0[ i] = (s0_vppn[18:9]==tlb_vppn[ i][18:9])
                        && (tlb_ps4MB[ i] || s0_vppn[8:0]==tlb_vppn[ i][8:0])
                        && ((s0_asid==tlb_asid[ i]) || tlb_g[ i]) && tlb_e[i];

        assign match1[ i] = (s1_vppn[18:9]==tlb_vppn[ i][18:9])
                        && (tlb_ps4MB[ i] || s1_vppn[8:0]==tlb_vppn[ i][8:0])
                        && ((s1_asid==tlb_asid[ i]) || tlb_g[ i]) && tlb_e[i];

        assign cond1[i] = ~tlb_g[i];
        assign cond2[i] = tlb_g[i];
        assign cond3[i] = s1_asid == tlb_asid[i];
        assign cond4[i] = (s1_vppn[18:9] == tlb_vppn[i][18:9])
                        & (tlb_ps4MB[i]||(s1_vppn[8:0] == tlb_vppn[i][8:0]));
        assign invtlb_mask[i] = invtlb_op == 0 && (cond1[i] || cond2[i])
                             || invtlb_op == 1 && (cond1[i] || cond2[i])
                             || invtlb_op == 2 && (cond2[i])
                             || invtlb_op == 3 && (cond1[i])
                             || invtlb_op == 4 && (cond1[i] && cond3[i])
                             || invtlb_op == 5 && (cond1[i] && cond3[i] && cond4[i])
                             || invtlb_op == 6 && ((cond2[i] || cond3[i]) && cond4[i]);
    end
endgenerate

encoder_16_4 encoder0(
    .in(match0),
    .out(s0_index)
);

encoder_16_4 encoder1(
    .in(match1),
    .out(s1_index)
);

assign s0_found = |match0;
assign s0_sel = tlb_ps4MB[s0_index] ? s0_vppn[8] : s0_va_bit12;//TODO
assign s0_ppn = s0_sel ? tlb_ppn1[s0_index] : tlb_ppn0[s0_index];
assign s0_ps = tlb_ps4MB[s0_index] ? 21 : 12;
assign s0_plv = s0_sel ? tlb_plv1[s0_index] : tlb_plv0[s0_index];
assign s0_mat = s0_sel ? tlb_mat1[s0_index] : tlb_mat0[s0_index];
assign s0_d = s0_sel ? tlb_d1[s0_index] : tlb_d0[s0_index];
assign s0_v = s0_sel ? tlb_v1[s0_index] : tlb_v0[s0_index];

assign s1_found = |match1;
assign s1_sel = tlb_ps4MB[s1_index] ? s1_vppn[8] : s1_va_bit12;//TODO
assign s1_ppn = s1_sel ? tlb_ppn1[s1_index] : tlb_ppn0[s1_index];
assign s1_ps = tlb_ps4MB[s1_index] ? 21 : 12;
assign s1_plv = s1_sel ? tlb_plv1[s1_index] : tlb_plv0[s1_index];
assign s1_mat = s1_sel ? tlb_mat1[s1_index] : tlb_mat0[s1_index];
assign s1_d = s1_sel ? tlb_d1[s1_index] : tlb_d0[s1_index];
assign s1_v = s1_sel ? tlb_v1[s1_index] : tlb_v0[s1_index];

//read
assign r_e       = tlb_e       [r_index];
assign r_vppn    = tlb_vppn    [r_index];
assign r_ps      = tlb_ps4MB   [r_index] ? 21 : 12;
assign r_asid    = tlb_asid    [r_index];
assign r_g       = tlb_g       [r_index];
assign r_ppn0    = tlb_ppn0    [r_index];
assign r_plv0    = tlb_plv0    [r_index];
assign r_mat0    = tlb_mat0    [r_index];
assign r_d0      = tlb_d0      [r_index];
assign r_v0      = tlb_v0      [r_index];
assign r_ppn1    = tlb_ppn1    [r_index];
assign r_plv1    = tlb_plv1    [r_index];
assign r_mat1    = tlb_mat1    [r_index];
assign r_d1      = tlb_d1      [r_index];
assign r_v1      = tlb_v1      [r_index];

//write
always @(posedge clk) begin
    if (we) begin
        tlb_e       [w_index] <= w_e;
        tlb_ps4MB   [w_index] <= w_ps == 21;
        tlb_vppn    [w_index] <= w_vppn;
        tlb_asid    [w_index] <= w_asid;
        tlb_g       [w_index] <= w_g;
        tlb_ppn0    [w_index] <= w_ppn0;
        tlb_plv0    [w_index] <= w_plv0;
        tlb_mat0    [w_index] <= w_mat0;
        tlb_d0      [w_index] <= w_d0;
        tlb_v0      [w_index] <= w_v0;
        tlb_ppn1    [w_index] <= w_ppn1;
        tlb_plv1    [w_index] <= w_plv1;
        tlb_mat1    [w_index] <= w_mat1;
        tlb_d1      [w_index] <= w_d1;
        tlb_v1      [w_index] <= w_v1;
    end
    else if(invtlb_valid) begin
        tlb_e                 <= ~invtlb_mask & tlb_e;
    end
end

endmodule

module encoder_16_4(
    input  wire [15:0] in,
    output wire [ 3:0] out
);

assign out = {4{in[0]}} & 0
           | {4{in[1]}} & 1
           | {4{in[2]}} & 2
           | {4{in[3]}} & 3
           | {4{in[4]}} & 4
           | {4{in[5]}} & 5
           | {4{in[6]}} & 6
           | {4{in[7]}} & 7
           | {4{in[8]}} & 8
           | {4{in[9]}} & 9
           | {4{in[10]}} & 10
           | {4{in[11]}} & 11
           | {4{in[12]}} & 12
           | {4{in[13]}} & 13
           | {4{in[14]}} & 14
           | {4{in[15]}} & 15;

endmodule