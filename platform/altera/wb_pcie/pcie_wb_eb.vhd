library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.pcie_wb_pkg.all;
use work.wishbone_pkg.all;
use work.etherbone_pkg.all;
use work.eb_internals_pkg.all;
use work.eb_hdr_pkg.all;

entity pcie_wb_eb is
  generic(
    g_family   : string  := "Arria V";
    g_fast_ack : boolean := true;
    sdb_addr   : t_wishbone_address;
    g_timeout_cycles : natural;
    g_fifo_size      : natural := 2048
  );
  port(
    clk125_i      : in  std_logic; -- 125 MHz, free running
    cal_clk50_i   : in  std_logic; --  50 MHz, shared between all PHYs
    
    -- Physical PCIe pins
    pcie_refclk_i : in  std_logic; -- 100 MHz, must not derive clk125_i or cal_clk50_i
    pcie_rstn_i   : in  std_logic; -- Asynchronous "clear sticky" PCIe pin
    pcie_rx_i     : in  std_logic_vector(3 downto 0);
    pcie_tx_o     : out std_logic_vector(3 downto 0);
    
    -- Commands from PC to FPGA
    master_clk_i  : in  std_logic;
    master_rstn_i : in  std_logic;
    master_o      : out t_wishbone_master_out;
    master_i      : in  t_wishbone_master_in;
    
    -- Command to PC from FPGA
    slave_clk_i   : in  std_logic := '0';
    slave_rstn_i  : in  std_logic := '1';
    slave_i       : in  t_wishbone_slave_in := cc_dummy_slave_in;
    slave_o       : out t_wishbone_slave_out;
    
    -- EB slave -- Commands from PC to FPGA
    eb_slv_master_clk_i  : in  std_logic;
    eb_slv_master_rstn_i : in  std_logic;
    eb_slv_master_o      : out t_wishbone_master_out;
    eb_slv_master_i      : in  t_wishbone_master_in;

    -- debug output
    r_cyc_o       : out std_logic;
    r_cyc_eb_o    : out std_logic
    
    );
end pcie_wb_eb;

architecture rtl of pcie_wb_eb is


  constant c_BAR_CFG    : std_logic_vector(2 downto 0) := "000";
  constant c_BAR_XWB    : std_logic_vector(2 downto 0) := "001";
  constant c_BAR_EBSLV  : std_logic_vector(2 downto 0) := "010";
  constant c_BAR_EBSLV2 : std_logic_vector(2 downto 0) := "011";
  constant c_BAR_EBSLV3 : std_logic_vector(2 downto 0) := "100";


  signal internal_wb_clk, internal_wb_rstn, stall : std_logic; 
  signal internal_wb_rstn_sync : std_logic_vector(3 downto 0) := (others => '0');
  
  signal rx_wb64_stb, rx_wb64_stall : std_logic;
  signal rx_wb32_stb, rx_wb32_stall : std_logic;
  signal rx_wb64_dat : std_logic_vector(63 downto 0);
  signal rx_wb32_dat : std_logic_vector(31 downto 0);
  signal rx_bar : std_logic_vector(2 downto 0);
  
  signal tx_rdy, tx_eop : std_logic;
  signal tx64_alloc, tx_wb64_stb : std_logic;
  signal tx32_alloc, tx_wb32_stb : std_logic;
  signal tx_wb64_dat : std_logic_vector(63 downto 0);
  signal tx_wb32_dat : std_logic_vector(31 downto 0);
  
  signal tx_alloc_mask : std_logic := '1'; -- Only pass every even tx32_alloc to tx64_alloc.
  
  signal wb_stb, wb_ack, wb_stall, wb_we, wb_err, wb_rty : std_logic;
  signal wb_sel     : std_logic_vector(3 downto 0);
  signal wb_bar     : std_logic_vector( 2 downto 0);  
  signal wb_adr     : std_logic_vector(63 downto 0);
  signal wb_dat_out : std_logic_vector(31 downto 0);
  signal wb_dat_in  : std_logic_vector(31 downto 0);

  signal wb_bar_adr : std_logic_vector( 3 downto 0);
  
  signal cfg_busdev : std_logic_vector(12 downto 0);
  
  -- Internal WB clock, PC->FPGA
  signal int_slave_i : t_wishbone_slave_in;
  signal int_slave_o : t_wishbone_slave_out;
  -- Internal WB clock: FPGA->PC
  signal int_master_o : t_wishbone_master_out;
  signal int_master_i : t_wishbone_master_in;
  signal ext_slave_o  : t_wishbone_slave_out;

  signal ctrl_bar_out : t_wishbone_slave_out;


  -- EB slave connections
  -- Internal WB clock, PC->FPGA
  signal eb_slv_int_slave_in  : t_wishbone_slave_in;
  signal eb_slv_int_slave_out : t_wishbone_slave_out;
  
  signal eb_pcie_slave_bar   : std_logic;
  
  -- Internal WB clock: FPGA->PC
  signal eb_slv_int_master_o : t_wishbone_master_out;
  signal eb_slv_int_master_i : t_wishbone_master_in;
  signal eb_slv_ext_slave_o  : t_wishbone_slave_out;
  
  signal eb_rx_fifo_master_o : t_wishbone_master_out;
  signal eb_rx_fifo_master_i : t_wishbone_master_in;
  
  signal eb_slv_int_slave_o  : t_wishbone_master_out;
  signal eb_slv_int_slave_i  : t_wishbone_master_in;
  
  signal pci_rx_fifo_master_o : t_wishbone_master_out;
  signal pci_rx_fifo_master_i : t_wishbone_master_in;
  
  signal eb_rx_stream_master_o  : t_wishbone_master_out;
  signal eb_rx_stream_master_i  : t_wishbone_master_in;
  
  -- control registers
  signal r_cyc   : std_logic;
  signal r_int   : std_logic := '0'; -- interrupt mask, starts=0
  signal r_addr  : std_logic_vector(31 downto 16);
  signal r_error : std_logic_vector(63 downto  0);
  
  signal r_cyc_max_tout    : unsigned(31 downto 0):= x"FFFFFFF0";
  signal cyc_max_tout      : unsigned(31 downto 0);
  signal r_xwb_op_max_time : unsigned(31 downto 0);
  
  
  
  signal r_cyc_tout_counter     : unsigned(31 downto 0);
  
  signal r_num_of_xwb_cycle_ops   : unsigned(15 downto 0);
  
  signal xwb_cyc      : std_logic;
  signal xwb_cyc_tout : std_logic;

  
  -- interrupt signals
  signal fifo_full, r_fifo_full, app_int_sts, app_msi_req : std_logic;
  
  
  -- debug signals
  
  signal r_time_ref_counter : unsigned(23 downto 0);
  
  
  signal issp_source : std_logic_vector(63 downto 0);
  signal issp_probe  : std_logic_vector(63 downto 0);   
  
    component issp_64i_64o is
        port (
            source     : out std_logic_vector(63 downto 0);                    -- source
            probe      : in  std_logic_vector(63 downto 0) := (others => 'X'); -- probe
            source_clk : in  std_logic                     := 'X'              -- clk
        );
    end component issp_64i_64o;  
  
  
begin

  pcie_phy : pcie_altera 
    generic map(
      g_family => g_family)
    port map(
      clk125_i      => clk125_i,
      cal_clk50_i   => cal_clk50_i,
      async_rstn    => master_rstn_i and slave_rstn_i,
      
      pcie_refclk_i => pcie_refclk_i,
      pcie_rstn_i   => pcie_rstn_i,
      pcie_rx_i     => pcie_rx_i,
      pcie_tx_o     => pcie_tx_o,

      cfg_busdev_o  => cfg_busdev,
      app_msi_req   => app_msi_req,
      app_int_sts   => app_int_sts,

      wb_clk_o      => internal_wb_clk,
      wb_rstn_i     => internal_wb_rstn,
      
      rx_wb_stb_o   => rx_wb64_stb,
      rx_wb_dat_o   => rx_wb64_dat,
      rx_wb_stall_i => rx_wb64_stall,
      rx_bar_o      => rx_bar,
      
      tx_rdy_o      => tx_rdy,
      tx_alloc_i    => tx64_alloc,
      
      tx_wb_stb_i   => tx_wb64_stb,
      tx_wb_dat_i   => tx_wb64_dat,
      tx_eop_i      => tx_eop);
  
  pcie_rx : pcie_64to32 port map(
    clk_i            => internal_wb_clk,
    rstn_i           => internal_wb_rstn,
    master64_stb_i   => rx_wb64_stb,
    master64_dat_i   => rx_wb64_dat,
    master64_stall_o => rx_wb64_stall,
    slave32_stb_o    => rx_wb32_stb,
    slave32_dat_o    => rx_wb32_dat,
    slave32_stall_i  => rx_wb32_stall);
  
  pcie_tx : pcie_32to64 port map(
    clk_i            => internal_wb_clk,
    rstn_i           => internal_wb_rstn,
    master32_stb_i   => tx_wb32_stb,
    master32_dat_i   => tx_wb32_dat,
    master32_stall_o => open,
    slave64_stb_o    => tx_wb64_stb,
    slave64_dat_o    => tx_wb64_dat,
    slave64_stall_i  => '0');
  
  pcie_logic : pcie_tlp port map(
    clk_i         => internal_wb_clk,
    rstn_i        => internal_wb_rstn,
    rx_wb_stb_i   => rx_wb32_stb,
    rx_wb_dat_i   => rx_wb32_dat,
    rx_wb_stall_o => rx_wb32_stall,
    rx_bar_i      => rx_bar,
    
    tx_rdy_i      => tx_rdy,
    tx_alloc_o    => tx32_alloc,
    tx_wb_stb_o   => tx_wb32_stb,
    tx_wb_dat_o   => tx_wb32_dat,
    tx_eop_o      => tx_eop,
    
    cfg_busdev_i  => cfg_busdev,
      
    wb_stb_o      => wb_stb,
    wb_adr_o      => wb_adr,
    wb_bar_o      => wb_bar,
    wb_we_o       => wb_we,
    wb_dat_o      => wb_dat_out,
    wb_sel_o      => wb_sel,
    wb_stall_i    => wb_stall,
    wb_ack_i      => wb_ack,
    wb_err_i      => wb_err,
    wb_rty_i      => wb_rty,
    wb_dat_i      => wb_dat_in);
    
    
    
--wb_bar_adr <= wb_bar & wb_adr(7);
    
--  -- select signals from bars back towards wb pci core master
--  p_wb_master_mux :  process(wb_bar_adr)
--  begin
--    case wb_bar_adr is
--        when "0001" => -- EB slave
--            wb_stall     <= eb_slv_int_slave_out.stall;
--            wb_err       <= eb_slv_int_slave_out.err  ;
--            wb_rty       <= eb_slv_int_slave_out.rty  ;  
--            wb_ack       <= eb_slv_int_slave_out.ack  ;
--            ctrl_bar_out.dat    <= eb_slv_int_slave_out.dat  ;
--        
--        when "0000" => -- control bar
--            wb_stall  <= '0';
--            wb_err    <= '0';
--            wb_rty    <= '0';  
--            wb_ack    <= ctrl_bar_out.ack;
--            ctrl_bar_out.dat <= ctrl_bar_out.dat;
--        
--        when others => -- XWB bar
--            wb_stall  <= int_slave_o.stall;
--            wb_err    <= int_slave_o.err  ;
--            wb_rty    <= int_slave_o.rty  ;
--            wb_ack    <= int_slave_o.ack  ;
--            ctrl_bar_out.dat <= int_slave_o.dat  ;
--
--     end case;
--  end process;
  
  p_wb_in_mux :  process(wb_bar, wb_adr)
  begin
    if (wb_bar = c_BAR_CFG and wb_adr(7) = '1') then
       wb_stall     <= eb_slv_int_slave_out.stall;
       wb_err       <= eb_slv_int_slave_out.err  ;
       wb_rty       <= eb_slv_int_slave_out.rty  ;  
       wb_ack       <= eb_slv_int_slave_out.ack  ;
       wb_dat_in    <= eb_slv_int_slave_out.dat  ;

    elsif wb_bar = c_BAR_XWB then
      wb_stall     <= int_slave_o.stall;
      wb_err       <= int_slave_o.err  ;
      wb_rty       <= int_slave_o.rty  ;
      wb_ack       <= int_slave_o.ack  ;
      wb_dat_in    <= int_slave_o.dat  ;

    else
      wb_stall  <= '0';
      wb_err    <= '0';
      wb_rty    <= '0';  
      wb_ack    <= ctrl_bar_out.ack;
      wb_dat_in <= ctrl_bar_out.dat;
    end if;
  end process;



  -- ###################################################################
  
  int_slave_i.we  <= wb_we     ;
  int_slave_i.dat <= wb_dat_out;
  int_slave_i.sel <= wb_sel    ;
  
  int_slave_i.stb <= wb_stb when wb_bar = c_BAR_XWB else '0';
  int_slave_i.cyc <= r_cyc;
  
  int_slave_i.adr(r_addr'range) <= r_addr;
  int_slave_i.adr(r_addr'right-1 downto 0)  <= wb_adr(r_addr'right-1 downto 0);  
    
  
  internal_wb_rstn <= internal_wb_rstn_sync(0);
  tx64_alloc <= tx32_alloc and tx_alloc_mask;
  
  alloc : process(internal_wb_clk)
  begin
    if rising_edge(internal_wb_clk) then
      internal_wb_rstn_sync <= (master_rstn_i and slave_rstn_i) & internal_wb_rstn_sync(internal_wb_rstn_sync'length-1 downto 1);
      
      if internal_wb_rstn = '0' then
        tx_alloc_mask <= '1';
      else
        tx_alloc_mask <= tx_alloc_mask xor tx32_alloc;
      end if;
    end if;
  end process;
  
  PC_to_FPGA_clock_crossing : xwb_clock_crossing 
    generic map(g_size => 32) port map(
    slave_clk_i    => internal_wb_clk,
    slave_rst_n_i  => internal_wb_rstn,
    slave_i        => int_slave_i,
    slave_o        => int_slave_o,
    master_clk_i   => master_clk_i, 
    master_rst_n_i => master_rstn_i,
    master_i       => master_i,
    master_o       => master_o);
  
  FPGA_to_PC_clock_crossing : xwb_clock_crossing
    generic map(g_size => 32) port map(
    slave_clk_i    => slave_clk_i,
    slave_rst_n_i  => slave_rstn_i,
    slave_i        => slave_i,
    slave_o        => ext_slave_o,
    master_clk_i   => internal_wb_clk, 
    master_rst_n_i => internal_wb_rstn,
    master_i       => int_master_i,
    master_o       => int_master_o);
  
  -- Do not wait for software acknowledgement
  fask_ack : if g_fast_ack generate
    slave_o.stall <= ext_slave_o.stall;
    slave_o.rty <= '0';
    slave_o.err <= '0';
    slave_o.dat <= (others => '0');
    
    fast_ack : process(slave_clk_i)
    begin
      if rising_edge(slave_clk_i) then
        slave_o.ack <= slave_i.cyc and slave_i.stb and not ext_slave_o.stall;
      end if;
    end process;
  end generate;
  
  -- Uses ack/err and dat from software
  slow_ack : if not g_fast_ack generate
    slave_o <= ext_slave_o;
  end generate;

  -- Notify the system when the FIFO is non-empty
  fifo_full <= int_master_o.cyc and int_master_o.stb;
  app_int_sts <= fifo_full and r_int; -- Classic interrupt until FIFO drained
  app_msi_req <= fifo_full and not r_fifo_full; -- Edge-triggered MSI
  
  int_master_i.rty <= '0';
  
  control : process(internal_wb_clk)
  begin
    if rising_edge(internal_wb_clk) then
      r_fifo_full <= fifo_full;
      
      -- Shift in the error register
      if int_slave_o.ack = '1' or int_slave_o.err = '1' or int_slave_o.rty = '1' then
          r_error <= r_error(r_error'length-2 downto 0) & (int_slave_o.err or int_slave_o.rty);
      end if;
  
-- XWB bar is muxed back to pcie in the p_wb_master_mux
--      if wb_bar = c_BAR_XWB then
--        wb_stall  <= int_slave_o.stall;
--        wb_err    <= int_slave_o.err  ;
--        wb_rty    <= int_slave_o.rty  ;
--        wb_ack    <= int_slave_o.ack;
--        wb_dat_in <= int_slave_o.dat;
--        
--      elsif wb_bar = c_BAR_CFG and wb_adr(7) = '1' then -- The EB Slave is targetted
--        wb_stall     <= eb_slv_int_slave_out.stall;
--        wb_err       <= eb_slv_int_slave_out.err  ;
--        wb_rty       <= eb_slv_int_slave_out.rty  ;
--        wb_ack       <= eb_slv_int_slave_out.ack  ;
--        wb_dat_in    <= eb_slv_int_slave_out.dat  ;
        
      if wb_bar = c_BAR_CFG and wb_adr(7) = '0' then -- The control BAR is targetted
        -- Feedback acks one cycle after strobe
        ctrl_bar_out.ack    <= wb_stb;
        
        -- Always output read result (even w/o stb or we)
        case wb_adr(6 downto 2) is
          when "00000" => -- 0x00 - Control register high
            ctrl_bar_out.dat(31) <= r_cyc;
            ctrl_bar_out.dat(30) <= '0';
            ctrl_bar_out.dat(29) <= r_int;
            ctrl_bar_out.dat(28 downto 0) <= (others => '0');
          when "00010" => -- 0x08 - Error flag high
            ctrl_bar_out.dat <= r_error(63 downto 32);
          when "00011" => -- 0x0C - Error flag low
            ctrl_bar_out.dat <= r_error(31 downto 0);
          when "00101" => -- 0x14 - Window offset low
            ctrl_bar_out.dat(r_addr'range) <= r_addr;
            ctrl_bar_out.dat(r_addr'right-1 downto 0) <= (others => '0');
          when "00111" => -- 0x1C - SDWB address low
            ctrl_bar_out.dat <= sdb_addr;
          
          when "01001" => -- 0x24 - Cycle Timeout setting
            ctrl_bar_out.dat <= std_logic_vector(r_cyc_max_tout);

          when "01010" => -- 0x28 - Max WB op time
            ctrl_bar_out.dat <= std_logic_vector(r_xwb_op_max_time);
            
          when "10000" => -- 0x40 - Master FIFO status & flags
            ctrl_bar_out.dat(31) <= fifo_full;
            ctrl_bar_out.dat(30) <= int_master_o.we;
            ctrl_bar_out.dat(29 downto 4) <= (others => '0');
            ctrl_bar_out.dat(3 downto 0) <= int_master_o.sel;
          when "10011" => -- 0x4C - Master FIFO adr low
            ctrl_bar_out.dat <= int_master_o.adr and c_pcie_msi.sdb_component.addr_last(31 downto 0);
          when "10101" => -- 0x54 - Master FIFO dat low
            ctrl_bar_out.dat <= int_master_o.dat;
          when others =>
            ctrl_bar_out.dat <= x"DEADC0DE";
        end case;
        
        -- Unless requested to by the PC, don't deque the FPGA->PC FIFO
        int_master_i.stall <= '1';
        int_master_i.ack <= '0';
        int_master_i.err <= '0';
        
        -- Is this a write to the register space?
        if wb_stb = '1' and wb_we = '1' then
          case wb_adr(6 downto 2) is
            when "00000" => -- Control register high
              if wb_sel(3) = '1' then
---                if wb_dat_out(30) = '1' then
---                  r_cyc <= wb_dat_out(31);
---                end if;
                if wb_dat_out(28) = '1' then
                  r_int <= wb_dat_out(29);
                end if;
              end if;
            when "00101" => -- Window offset low
              if wb_sel(3) = '1' then
                r_addr(31 downto 24) <= wb_dat_out(31 downto 24);
              end if;
              if wb_sel(2) = '1' then
                r_addr(24 downto 16) <= wb_dat_out(24 downto 16);
              end if;
            when "10000" => -- Master FIFO status & flags
              if wb_sel(0) = '1' then
                case wb_dat_out(1 downto 0) is
                  when "00" => null;
                  when "01" => int_master_i.stall <= '0';
                  when "10" => int_master_i.ack <= '1';
                  when "11" => int_master_i.err <= '1';
                end case;
              end if;
              
            when "10101" => -- Master FIFO data low
              int_master_i.dat <= wb_dat_out;

            when "01001" => -- 0x24 - Cycle Timeout setting
              r_cyc_max_tout <= unsigned(wb_dat_out);
              
            when others => null;
          end case;
        end if;
      end if;
    end if;
  end process;
  
  
  
  -- control maximum cycle timeout from SW or ISSP
  cyc_max_tout <= unsigned(issp_source(63 downto 32)) when issp_source(31) = '1' else r_cyc_max_tout;
  
  -- xwb cycle timeout
  xwb_cyc_tout <= '0' when (r_cyc_tout_counter < cyc_max_tout)  else '1';
  
  p_cyc_timeout: process(internal_wb_clk)
  begin
    if rising_edge(internal_wb_clk) then
      if internal_wb_rstn = '0' then
        r_cyc <= '0';
        r_cyc_tout_counter <= (others => '0');
        r_xwb_op_max_time  <= (others => '0');
      else
        -- host controls xwb cycle
        if (wb_bar = c_BAR_CFG and wb_adr(7 downto 2) = "000000" and
            wb_stb = '1' and wb_we = '1' and 
            wb_sel(3) = '1' and wb_dat_out(30) = '1') 
        then
          r_cyc <= wb_dat_out(31);
        -- ohterwise if cycle timeout occured then drop cycle  
        elsif xwb_cyc_tout = '1' then 
          r_cyc <= '0';
        else -- hold
          r_cyc <= r_cyc;
        end if;
        
        -- cycle time out counter
        -- xwb cycle not active or we have some activity from host to xwb
        if (   r_cyc           = '0' 
            or ( wb_stb = '1' and wb_bar = c_BAR_XWB )
            or int_slave_o.ack = '1' 
            or int_slave_o.err = '1' 
            or int_slave_o.rty = '1' 
            or xwb_cyc_tout = '1') 
        then
          r_cyc_tout_counter <= (others => '0');
        -- count up when xwb cycle opened
        else
          r_cyc_tout_counter <= r_cyc_tout_counter + 1;
        end if;
        
        -- 0x28 - reset Max WB op time
        if ((wb_adr(7 downto 2) = "001010" and wb_bar = c_BAR_CFG and 
            wb_stb = '1' and wb_we = '1') or issp_source(30) = '1')
        then
          r_xwb_op_max_time <= (others => '0');
        
        -- log max time in the cycle withtout activity from host 
        elsif r_cyc_tout_counter > r_xwb_op_max_time then
          r_xwb_op_max_time <= r_cyc_tout_counter;
        else -- hold
          r_xwb_op_max_time <= r_xwb_op_max_time;
        end if;
        
      end if;
    end if;
  end process;
  
  
  -- aux counter for time reference in SignalTap when using conditional storage
  -- option to enable triggers for reseting it, otherwise it just counts up
  p_trc_wb: process(internal_wb_clk)
  variable r_cyc_dly     : std_logic;
    variable r_wb_stb_dly  : std_logic;
  begin
    if rising_edge(internal_wb_clk) then
        -- if enabled by issp, reset counter when WB cycle is raised
        if (issp_source(1) = '1' and r_cyc  = '1' and r_cyc_dly    = '0') or 
           (issp_source(2) = '1' and wb_stb = '1' and r_wb_stb_dly = '0')
        then
            r_time_ref_counter <= (others => '0');
        else
            r_time_ref_counter <= r_time_ref_counter + 1;
        end if;
        
        r_cyc_dly     := r_cyc;
        r_wb_stb_dly  := wb_stb;
    end if;
  end process;
 
 
 r_cyc_o <= r_cyc;
 
  
  -- In-Syster source and probe   
  issp_bar0_regs : issp_64i_64o
    port map (
      source     => issp_source,     --    sources.source
      probe      => issp_probe,      --     probes.probe
      source_clk => internal_wb_clk  -- source_clk.clk
    );
  
issp_probe(0) <= r_cyc;
issp_probe(1) <= r_int;    
issp_probe(2) <= fifo_full  ;
issp_probe(3) <= app_int_sts;
issp_probe(4) <= int_master_o.we;

issp_probe(5) <= '1' when r_time_ref_counter = x"000000" else '0';


issp_probe(6) <= xwb_cyc_tout;


issp_probe(31 downto 16) <= (others => '0');
issp_probe(63 downto 32) <= std_logic_vector(r_xwb_op_max_time);  
  
  --#####################################################################
  -- everything related to eb_slave
  
-- ################################################################

eb_slv_int_slave_in.we  <= wb_we ;
eb_slv_int_slave_in.adr <= wb_adr(eb_slv_int_slave_in.adr'range);
eb_slv_int_slave_in.dat <= wb_dat_out;
eb_slv_int_slave_in.sel <= wb_sel; 
eb_slv_int_slave_in.stb <= wb_stb  when (wb_bar = c_BAR_CFG and wb_adr(7) = '1') else '0';


eb_slave: entity work.eb_pci_slave
  generic map (
    g_fast_ack          => false,
    g_sdb_address       => sdb_addr,
    g_timeout_cycles    => g_timeout_cycles,
    g_fifo_size         => g_fifo_size
  )
  port map (
    clk_wb_i      => internal_wb_clk     , -- in  std_logic; -- clock from PCI WB side
    rstn_wb_i     => internal_wb_rstn    , -- in  std_logic;
    clk_xwb_i     => eb_slv_master_clk_i , -- in  std_logic; -- clock from XWB side
    rstn_xwb_i    => eb_slv_master_rstn_i, -- in  std_logic;

    -- Command from PC to EB slave
    slave_i       => eb_slv_int_slave_in , --  in  t_wishbone_slave_in := cc_dummy_slave_in;
    slave_o       => eb_slv_int_slave_out, --  out t_wishbone_slave_out;
    
    -- Commands from EB slave to XWB crossbar
    master_o      => eb_slv_master_o, --  out t_wishbone_master_out;
    master_i      => eb_slv_master_i,  --  in  t_wishbone_master_in
    
    phy_cyc_o     => r_cyc_eb_o
    );
  
end rtl;
