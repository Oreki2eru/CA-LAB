module bridge(
    input         aclk,
    input         aresetn,
    // 读请求通道
    output [ 3:0] arid   , // 读请求ID（取值置位0；取数置1）
    output [31:0] araddr , // 读请求地址
    output [ 7:0] arlen  , // 读请求传输长度（数据传输拍数）（固定为0）
    output [ 2:0] arsize , // 读请求传输大小（数据传输每拍的字节数）
    output [ 1:0] arburst, // 传输类型（固定为0b01）
    output [ 1:0] arlock , // 原子锁（固定为0）
    output [ 3:0] arcache, // Cache属性（固定为0）
    output [ 2:0] arprot , // 保护属性（固定为0）
    output        arvalid, // 读请求地址有效
    input         arready, // 读请求地址握手信号
    // 读响应通道
    input [ 3:0]  rid    , // 读请求ID号，同一请求rid与arid一致
    input [31:0]  rdata  , // 读请求读出的数据
    input [ 1:0]  rresp  , // 读请求是否完成 [可忽略]
    input         rlast  , // 读请求最后一拍数据的指示信号 [可忽略]
    input         rvalid , // 读请求数据有效
    output        rready , // Master端准备好接受数据
    // 写请求通道
    output [ 3:0] awid   , // 写请求的ID号（固定为1）
    output [31:0] awaddr , // 写请求的地址
    output [ 7:0] awlen  , // 写请求传输长度（拍数）（固定为0）
    output [ 2:0] awsize , // 写请求传输每拍字节数
    output [ 1:0] awburst, // 写请求传输类型（固定为0b01）
    output [ 1:0] awlock , // 原子锁（固定为0）
    output [ 3:0] awcache, // Cache属性（固定为0）
    output [ 2:0] awprot , // 保护属性（固定为0）
    output        awvalid, // 写请求地址有效
    input         awready, // Slave端准备好接受地址传输   
    // 写数据通道
    output [ 3:0] wid    , // 写请求的ID号（固定为1）
    output [31:0] wdata  , // 写请求的写数据
    output [ 3:0] wstrb  , // 写请求字节选通位
    output        wlast  , // 写请求的最后一拍数据的指示信号（固定为1）
    output        wvalid , // 写数据有效
    input         wready , // Slave端准备好接受写数据传输   
    // 写响应通道
    input  [ 3:0] bid    , // 写请求的ID号            [可忽略]
    input  [ 1:0] bresp  , // 写请求完成信号          [可忽略]
    input         bvalid , // 写请求响应有效
    output        bready , // Master端准备好接收响应信号
    // icache rd interface
    input               	icache_rd_req,
    input   	[ 2:0]      icache_rd_type,
    input   	[31:0]      icache_rd_addr,
    output              	icache_rd_rdy,			// icache_addr_ok
    output              	icache_ret_valid,	// icache_data_ok
	output					icache_ret_last,
    output  	[31:0]      icache_ret_data,
    // data sram interface
    input               	data_sram_req,
    input               	data_sram_wr,
    input   	[ 1:0]      data_sram_size,
    input   	[31:0]      data_sram_addr,
    input   	[31:0]      data_sram_wdata,
    input   	[ 3:0]      data_sram_wstrb,
    output                  data_sram_addr_ok,
    output                  data_sram_data_ok,
    output      [31:0]      data_sram_rdata
);

reg [4:0] r_state;
reg [4:0] r_next_state;
parameter R_FREE = 4'b0001;
parameter R_SEND = 4'b0010;
parameter R_RECV = 4'b0100;

reg [4:0] w_state;
reg [4:0] w_next_state;
parameter W_FREE = 4'b0001;
parameter W_SEND = 4'b0010;
parameter W_RECV = 4'b0100;

reg    [ 3:0] arid_r;
reg    [31:0] araddr_r;
reg    [ 2:0] arsize_r;
reg           arvalid_r;
reg           rready_r;

assign arid    = arid_r; // 读请求ID（取值置位0；取数置1）
assign araddr  = araddr_r; // 读请求地址
assign arlen   = data_sram_req & ~data_sram_wr ?  8'b0 : {6'b0,{2{icache_rd_type[2]}}}/*8'b0*/; // 读请求传输长度（数据传输拍数）（固定为0）
assign arsize  = arsize_r; // 读请求传输大小（数据传输每拍的字节数）
assign arburst = 2'b1; // 传输类型（固定为0b01）
assign arlock  = 2'b0; // 原子锁（固定为0）
assign arcache = 4'b0; // Cache属性（固定为0）
assign arprot  = 3'b0; // 保护属性（固定为0）
assign arvalid = arvalid_r; // 读请求地址有效
// 读响应通道
assign rready  = rready_r; // Master端准备好接受数据

always @(posedge aclk) begin
    if(~aresetn) begin
        r_state <= R_FREE;
        arid_r <= 4'b0;
        araddr_r <= 32'b0;
        arsize_r <= 3'b0;
        arvalid_r <= 1'b0;
        rready_r <= 1'b0;
    end
    else begin
        case(r_state)
        R_FREE: begin
            if(/*inst_sram_req*/icache_rd_req || data_sram_req && ~data_sram_wr) begin
                r_state <= R_SEND;
                arid_r <= {3'b0, data_sram_req && ~data_sram_wr};
                araddr_r <= (data_sram_req && ~data_sram_wr) ? data_sram_addr : icache_rd_addr/*inst_sram_addr*/;
                arsize_r <= {1'b0, (data_sram_req && ~data_sram_wr) ? data_sram_size : 2'b10};
                arvalid_r <= 1'b1;
            end
        end
        R_SEND: begin
            if(arready) begin
                r_state <= R_RECV;
                arvalid_r <= 1'b0;
                rready_r <= 1'b1;
            end
        end
        R_RECV: begin
            if(rvalid & rlast) begin
                r_state <= R_FREE/*R_END*/;
                rready_r <= 1'b0;
            end
            else if(rvalid) begin
                r_state <= R_RECV;
                rready_r <= 1'b1;
            end
        end
        /*
        R_END: begin
            r_state = R_FREE;
        end
        */
        endcase
    end
end

reg [31:0] awaddr_r;
reg [ 2:0] awsize_r;
reg        awvalid_r;
reg [31:0] wdata_r;
reg [ 3:0] wstrb_r;
reg        wvalid_r;
reg        bready_r;

// 写请求通道
assign awid    = 1'b1; // 写请求的ID号（固定为1）
assign awaddr  = awaddr_r; // 写请求的地址
assign awlen   = 8'b0; // 写请求传输长度（拍数）（固定为0）
assign awsize  = awsize_r; // 写请求传输每拍字节数
assign awburst = 2'b1; // 写请求传输类型（固定为0b01）
assign awlock  = 2'b0; // 原子锁（固定为0）
assign awcache = 4'b0; // Cache属性（固定为0）
assign awprot  = 3'b0; // 保护属性（固定为0）
assign awvalid = awvalid_r; // 写请求地址有效
// 写数据通道
assign wid     = 4'b1; // 写请求的ID号（固定为1）
assign wdata   = wdata_r; // 写请求的写数据
assign wstrb   = wstrb_r; // 写请求字节选通位
assign wlast   = 1'b1; // 写请求的最后一拍数据的指示信号（固定为1）
assign wvalid  = wvalid_r; // 写数据有效
// 写响应通道
assign bready  = bready_r; // Master端准备好接收响应信号

reg awready_r;
reg wready_r;

always @(posedge aclk) begin
    if(~aresetn) begin
        awready_r <= 1'b0;
        wready_r <= 1'b0;
    end
    else begin
        if((awready || awready_r) && (wready || wready_r)) begin
            awready_r <= 1'b0;
            wready_r <= 1'b0;
        end
        else begin
            if(awvalid && awready) begin
                awready_r <= 1'b1;
            end
            if(wvalid && wready) begin
                wready_r <= 1'b1;
            end
        end
    end
end

always @(posedge aclk) begin
    if(~aresetn) begin
        w_state <= W_FREE;
        awaddr_r <= 32'b0;
        awsize_r <= 3'b0;
        awvalid_r <= 1'b0;
        wdata_r <= 32'b0;
        wstrb_r <= 4'b0;
        wvalid_r <= 1'b0;
        bready_r <= 1'b0;
    end
    else begin
        case(w_state)
        W_FREE: begin
            if(data_sram_req && data_sram_wr) begin
                w_state <= W_SEND;
                awaddr_r <= data_sram_addr;
                awsize_r <= {1'b0, data_sram_size};
                wdata_r <= data_sram_wdata;
                wstrb_r <= data_sram_wstrb;
                awvalid_r <= 1'b1;
                wvalid_r <= 1'b1;
            end
        end
        W_SEND: begin
            if((awready || awready_r) && (wready || wready_r)) begin
                w_state <= W_RECV;
                //awvalid_r <= 1'b0;
                //wvalid_r <= 1'b0;
                bready_r <= 1'b1;
            end
            if(awready) begin
                awvalid_r <= 1'b0;
            end
            if(wready) begin
                wvalid_r <= 1'b0;
            end
        end
        W_RECV: begin
            if(bvalid) begin
                w_state <= W_FREE;
                bready_r <= 1'b0;
            end
        end
        endcase
    end
end

always @(posedge aclk) begin
    if(~aresetn) begin
        //r_state <= R_FREE;
        //w_state <= W_FREE;
    end
    else begin
        //r_state <= r_next_state;
        //w_state <= w_next_state;
    end
end

//addrok应该遵循这样的规定，即datasram优先，只要空闲立即向datasram或者instsram发addrok信号，目的是不给它们修改的机会
assign icache_rd_rdy = (~data_sram_req || data_sram_wr) && (r_state == R_FREE) && icache_rd_req;// icache_addr_ok
assign icache_ret_data = rdata;
assign icache_ret_valid = ~arid_r[0] && rvalid && rready; // icache_data_ok
assign icache_ret_last = ~arid_r[0] && rvalid && rready && rlast; //rlast

assign data_sram_addr_ok = ~data_sram_wr && (r_state == R_FREE) && data_sram_req || data_sram_wr && (w_state == W_FREE) && data_sram_req;
assign data_sram_data_ok = arid_r[0] && rvalid && rready || bvalid && bready;
assign data_sram_rdata = rdata;
//dataok应该遵循这样的规定，即
//只允许一个已完成请求握手但数据尚未返回的读事务

endmodule