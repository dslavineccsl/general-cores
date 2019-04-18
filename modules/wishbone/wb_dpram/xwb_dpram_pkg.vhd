--! @file io_control_pkg.vhd
--! @brief Control unit for bidirectional IO and more
--! @author CSCO-TG <csco-tg@gsi.de>
--!
--! Copyright (C) 2015 GSI Helmholtz Centre for Heavy Ion Research GmbH 
--!
--------------------------------------------------------------------------------
--! This library is free software; you can redistribute it and/or
--! modify it under the terms of the GNU Lesser General Public
--! License as published by the Free Software Foundation; either
--! version 3 of the License, or (at your option) any later version.
--!
--! This library is distributed in the hope that it will be useful,
--! but WITHOUT ANY WARRANTY; without even the implied warranty of
--! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
--! Lesser General Public License for more details.
--!  
--! You should have received a copy of the GNU Lesser General Public
--! License along with this library. If not, see <http://www.gnu.org/licenses/>.
---------------------------------------------------------------------------------
-- Libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.wishbone_pkg.all;
use work.monster_pkg.all;


package xwb_dpram_pkg is

 
  constant c_xwb_dpram_dsl_sdb : t_sdb_device := (
    abi_class     => x"0000", -- undocumented device
    abi_ver_major => x"00",
    abi_ver_minor => x"00",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"7", -- 8/16/32-bit port granularity
    sdb_component => (
    addr_first    => x"0000000000000000",
    addr_last     => x"0000000000000fff",
    product => (
    vendor_id     => x"0000000000000651",
    device_id     => x"10C057FF",
    version       => x"00000001",
    date          => x"20190118",
    name          => "DPRAM-USER-DSL     "))
    );
  


--component xwb_dpram is
--  generic(
--    g_size                  : natural := 16384;
--    g_init_file             : string  := "";
--    g_must_have_init_file   : boolean := false;
--    g_slave1_interface_mode : t_wishbone_interface_mode;
--    g_slave2_interface_mode : t_wishbone_interface_mode;
--    g_slave1_granularity    : t_wishbone_address_granularity;
--    g_slave2_granularity    : t_wishbone_address_granularity
--    );
--  port(
--    clk_sys_i : in std_logic;
--    rst_n_i   : in std_logic;
--
--    slave1_i : in  t_wishbone_slave_in;
--    slave1_o : out t_wishbone_slave_out;
--    slave2_i : in  t_wishbone_slave_in;
--    slave2_o : out t_wishbone_slave_out
--    );
--end component;

  
end package;
