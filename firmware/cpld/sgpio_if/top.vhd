-- hacktv - Analogue video transmitter for the HackRF                    
--=======================================================================
-- Copyright 2019 Philip Heron <phil@sanslogic.co.uk>
-- Author: Matthew Millman <inaxeon@hotmail.com>
--                                                                       
-- This program is free software: you can redistribute it and/or modify  
-- it under the terms of the GNU General Public License as published by  
-- the Free Software Foundation, either version 3 of the License, or     
-- (at your option) any later version.                                   
--                                                                       
-- This program is distributed in the hope that it will be useful,       
-- but WITHOUT ANY WARRANTY; without even the implied warranty of        
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         
-- GNU General Public License for more details.                          
--                                                                       
-- You should have received a copy of the GNU General Public License     
-- along with this program.  If not, see <http://www.gnu.org/licenses/>. 

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_unsigned.all;

library UNISIM;
use UNISIM.vcomponents.all;

entity top is
    Port(
        HOST_DATA       : in    std_logic_vector(7 downto 0);
        HOST_CAPTURE    : out   std_logic;
        HOST_CLOCK      : out   std_logic;
        HOST_SYNC_EN    : in    std_logic;
        TCXO_IN         : in    std_logic;
        HOST_DISABLE    : in    std_logic;
        OUTPUT_ENABLED  : in    std_logic; -- Same as the HOST_DIRECTION signal. As HackDAC is output only HOST_DIRECTION = 1 means we're running.

        DA              : in    std_logic_vector(7 downto 0);
        DD              : out   std_logic_vector(9 downto 0);
        VDAC            : out   std_logic_vector(15 downto 0);
        VDAC_CLK        : out   std_logic;
        SYNC_OUT        : out   std_logic
    );

end top;

architecture Behavioral of top is
    signal codec_clk_rx_i : std_logic;
    signal codec_clk_tx_i : std_logic;
    signal dac_data_o : std_logic_vector(9 downto 0);
    signal host_clk_i : std_logic;
    signal host_data_enable_i : std_logic;
    signal host_data_capture_o : std_logic;
    signal host_sync_enable : std_logic := '0';
    signal host_sync_o : std_logic := '0';
    signal host_sync_i : std_logic := '0';
    signal host_sync_latched : std_logic := '0';
    signal dac_clock_o : std_logic := '0';
begin
    
    ------------------------------------------------
    -- HackRF DAC. Now completely unused.
     
    DD(9 downto 0) <= (others => '0');
    
    ------------------------------------------------
    -- Clocks
    
    host_clk_i <= TCXO_IN;
    HOST_CLOCK <= host_clk_i; -- To SGPIO 11 (Reconfigured as SGPIO clock input for HackDAC)
     
    ------------------------------------------------
    -- SGPIO interface
    
    HOST_CAPTURE <= host_data_capture_o;
    host_sync_enable <= HOST_SYNC_EN;
    host_sync_i <= '1';
    host_data_enable_i <= not HOST_DISABLE;
     
    ------------------------------------------------
        
    process(host_clk_i)
    begin
        if falling_edge(host_clk_i) then
               codec_clk_tx_i <= dac_clock_o;
               -- HackDAC Clock. NOTE: AD768 samples on rising edge. Latching here ends up with the correct timing.
               -- i.e. I and Q bytes in have both been latched and are stable
               VDAC_CLK <= not dac_clock_o;
        end if;
    end process;
     
    process(host_clk_i)
    begin
        if rising_edge(host_clk_i) then
            -- Generate DAC clock (TCXO / 2)
            dac_clock_o <= not dac_clock_o;
            codec_clk_rx_i <= dac_clock_o;
            -- Data latching is moved to the rising edge as the delay in getting the clock through the CPLD
            -- down to the SGPIO means that it ends up better to sample at this time rather than falling.
            if OUTPUT_ENABLED = '1' then
                if codec_clk_tx_i = '1' then
                    -- HackDAC 1-bit sync
                    SYNC_OUT <= not HOST_DATA(7);
                    -- HackDAC DAC MSBs
                    VDAC(15 downto 9) <= HOST_DATA(6 downto 0);
                else
                    -- HackDAC DAC LSBs
                    VDAC(8 downto 1) <= HOST_DATA;
                    VDAC(0) <= '0';
                end if;
            else
                -- HackTV not running. Zero outputs.
                SYNC_OUT <= '0'; -- 0V
                VDAC(15 downto 0) <= X"C000"; -- 0V
            end if;
        end if;
    end process;
    
    process (host_data_enable_i, host_sync_i)
    begin
        host_sync_o <= host_data_enable_i;
        if host_data_enable_i = '1' then
            if rising_edge(host_sync_i) then
                host_sync_latched <= host_sync_i;
            end if;
        else
            host_sync_latched <= '0';
        end if;
    end process;
    
    process(host_clk_i)
    begin
        if rising_edge(host_clk_i) then
             if codec_clk_tx_i = '1' then
                  host_data_capture_o <= host_data_enable_i and (host_sync_latched or not host_sync_enable);
             end if;
        end if;
    end process;
    
end Behavioral;