/**
  ******************************************************************************
  * Author             : Siraj Muhammad <sirajmuhammad@ou.edu>
  * File Name          : flash.c
  * Description        : Flash memory configuration and IO functions
  ******************************************************************************
  *
  * Copyright (c) 2018 - 2019 The University of Oklahoma - WECAD Center
  * All rights reserved.
  *
  ******************************************************************************
  */
#define NRF_LOG_MODULE_NAME flash   // This is the log prefix of this module

/******************************************************************************
 Header Files Includes
 *****************************************************************************/
#include "common.h"
#include "flash.h"
#include "ivccs3g.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"
#include "detection_alg.h"
#include "nrfx_log.h"
#include "math.h"
#include "configs.h"
#include "nrf_delay.h"
#include <stdlib.h>
#include "bluetooth.h"

NRF_LOG_MODULE_REGISTER();      // For log module to take the prefix effective

/******************************************************************************
 Definitions
*****************************************************************************/
#define FLASH_UNIT_SIZE         0x800000
#define MAX_FLASH_SPACE         FLASH_UNIT_SIZE * 4
#define GET_FLASH_UNIT(addr)    (floor((float)addr / (float)FLASH_UNIT_SIZE) + 1)
#define GET_LOCAL_ADDR(addr)    (addr % FLASH_UNIT_SIZE)

#define WAIT_FOR_WIP()                                                  \
{                                                                       \
    uint32_t time_out = 0xFFFFFF;                                       \
    while((nrfx_qspi_mem_busy_check() != NRFX_SUCCESS) && time_out)     \
    {                                                                   \
        wdt_feed();                                                     \
        time_out--;                                                     \
    };                                                                  \
    if (!time_out)                                                      \
    {                                                                   \
        NRFX_LOG_ERROR("QSPI WIP timed out.");                          \
        return NRFX_ERROR_TIMEOUT;                                      \
    }                                                                   \
}

/******************************************************************************
      Variables Declarations
******************************************************************************/
// Flash Variables
static uint32_t   current_global_addr = 0x00000000;
static uint8_t    RAM_buff[512] = {0};
static uint16_t   RAM_buff_size = 0;

/******************************************************************************
      Functions Declarations
******************************************************************************/
static ret_code_t flash_init(iVCCS_SF_array_unit sf_unit);
static ret_code_t flash_uninit(void);
static ret_code_t flash_write_bytes(char* buffer, size_t len, iVCCS_SF_array_unit Flash_Unit, uint32_t local_address);

/******************************************************************************
      Functions Definitions
******************************************************************************/
static ret_code_t flash_init(iVCCS_SF_array_unit sf_unit)
{
    ret_code_t err_code = NRFX_SUCCESS;
    uint8_t cs_pin = iVCCS_QSPI_SF1_CS;

    // Enable the corresponding flash power pin
    switch(sf_unit)
    {
        case iVCCS_SF1:
            nrfx_gpiote_out_set(iVCCS_QSPI_SF1_PWR);
            cs_pin = iVCCS_QSPI_SF1_CS;
            break;
        case iVCCS_SF2:
            nrfx_gpiote_out_set(iVCCS_QSPI_SF2_PWR);
            cs_pin = iVCCS_QSPI_SF2_CS;
            break;
        case iVCCS_SF3:
            nrfx_gpiote_out_set(iVCCS_QSPI_SF3_PWR);
            cs_pin = iVCCS_QSPI_SF3_CS;
            break;
        case iVCCS_SF4:
            nrfx_gpiote_out_set(iVCCS_QSPI_SF4_PWR);
            cs_pin = iVCCS_QSPI_SF4_CS;
            break;
        default:
            // Default case is the first flash unit
            nrfx_gpiote_out_set(iVCCS_QSPI_SF1_PWR);
            break;
    }

    nrf_delay_ms(5);

    // Configure QSPI
    nrfx_qspi_config_t qspi_config = NRFX_QSPI_DEFAULT_CONFIG;
    qspi_config.pins.csn_pin = cs_pin;
    err_code = nrfx_qspi_init(&qspi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    // Configure flash memory and switch to QSPI mode
    nrf_qspi_cinstr_conf_t cinstr_cfg =
    {
        .opcode    = FLASH_RSTEN,
        .length    = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait   = true,
        .wren      = true
    };

    // Send reset enable
    err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    // Send reset command
    cinstr_cfg.opcode = FLASH_RST;
    err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    // Switch to QSPI mode
    uint8_t param = FLASH_ENABLE_QE;
    uint8_t resp[2] = {0};
    cinstr_cfg.opcode = FLASH_WRSR;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
    err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, &param, resp);
    APP_ERROR_CHECK(err_code);

    WAIT_FOR_WIP();

    NRFX_LOG_DEBUG("QSPI and unit %d is initialized successfully.", sf_unit);

    return err_code;
}

static ret_code_t flash_uninit(void)
{
    if (!NRF_QSPI->ENABLE) return NRF_ERROR_MODULE_NOT_INITIALIZED;
    // Uninitialize QSPI peripheral
    nrfx_qspi_uninit();
    // Switch power off flash array
    nrfx_gpiote_out_clear(iVCCS_QSPI_SF1_PWR);
    nrfx_gpiote_out_clear(iVCCS_QSPI_SF2_PWR);
    nrfx_gpiote_out_clear(iVCCS_QSPI_SF3_PWR);
    nrfx_gpiote_out_clear(iVCCS_QSPI_SF4_PWR);

    NRFX_LOG_DEBUG("QSPI and all units uninitialized successfully.");

    return NRFX_SUCCESS;
}

/**
* @brief  Write data to specific unit and address
* @param  buffer: where timestamp data is stored
* @param  len: number of bytes to write
* @param  Flash_Unit: Flash unit that data is being written on
* @param  local_address: address on flash unit where data is being written to
* @return Error code.
*/
static ret_code_t flash_write_bytes(char* buffer, size_t len, iVCCS_SF_array_unit sf_unit, uint32_t local_address)
{
    ret_code_t err_code;

    // Configure the Flash Unit initialized
    err_code = flash_init(sf_unit);
    APP_ERROR_CHECK(err_code);

    // Erase the sector first. If the address points to the beginning of a sector, erase it
    // Every 16 pages form a sector.
    if (local_address % (16*256) == 0)
    {
        err_code = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, local_address);
        APP_ERROR_CHECK(err_code);

        // Wait for WIP bit
        WAIT_FOR_WIP();
    }

    // Write from buffer to flash
    err_code = nrfx_qspi_write(buffer, len, local_address);
    APP_ERROR_CHECK(err_code);

    // Wait for WIP bit
    WAIT_FOR_WIP();

    NRFX_LOG_DEBUG("%d bytes of data is written to unit: %d address: 0x%08x.", len, sf_unit, local_address);
    
    // Uninitialize the unit
    flash_uninit();

    return err_code;
}


/**
 * @brief  Buffer data and write them to flash.
 * @param  data: Pointer to data array.
 * @return Error code.
 */
ret_code_t flash_write(char* data)
{
    if (!ivccs_config.data_collect_enable) return NRFX_ERROR_INVALID_STATE;
    if (current_global_addr >= MAX_FLASH_SPACE) return NRFX_ERROR_NO_MEM;

    ret_code_t err_code = NRFX_SUCCESS;
    uint32_t local_address;
    iVCCS_SF_array_unit sf_unit;
    size_t length = strlen(data);

    // Copying data into buffer (RAM)
    if (RAM_buff_size+length > 512)
    {
        NRFX_LOG_ERROR("Data exceeds available space in RAM buffer!");
        return NRFX_ERROR_NO_MEM;
    }
    memcpy(RAM_buff + RAM_buff_size, (uint8_t*)data, length);

    // Change buffer size accordingly as data size increases each loop
    RAM_buff_size += length;

    // For an exact page in flash, we need buffer of 256 bytes
    // We write to a page when OTH is not set... that is when no vehicle detection is in process.
    // Or, if signature collection is enabled, write anyway because otherwise buffer overflows and causes problems.
    if (RAM_buff_size >= 256 && (ivccs_config.data_sig_collect || !OTH_INT_Flag))
    {
        // Calculate sf_unit to be used and the local address in the sf_unit
        sf_unit = GET_FLASH_UNIT(current_global_addr);
        local_address = GET_LOCAL_ADDR(current_global_addr);

        // Write a page to the flash
        err_code = flash_write_bytes(RAM_buff, 256, sf_unit, local_address);
        if (err_code == NRFX_SUCCESS)
        {
            // Shift remaining data to the start of the buffer
            memmove(RAM_buff, RAM_buff + 256, RAM_buff_size - 256);
            // Adjust the buffer size
            RAM_buff_size -= 256;
            // Increase the global address pointer
            current_global_addr += 256;

            if (current_global_addr >= MAX_FLASH_SPACE)
            {
                char msg[] = "Flash storage is full!\r\n";
                bluetooth_send(msg);
                NRFX_LOG_WARNING("Storage is full!");
            }
        }
        else
        {
            NRFX_LOG_ERROR("flash_write_bytes() returned: %d", err_code);
            return err_code;
        }
    }
    return err_code;
}

/**
 * @brief  Move remaining data in RAM buffers to flash.
 * @return Error code.
 */
ret_code_t flash_flush()
{
    if (RAM_buff_size == 0) return NRFX_SUCCESS;

    ret_code_t err_code;
    iVCCS_SF_array_unit sf_unit;
    uint32_t local_address;

    // Calculate sf_unit to be used and the local address in the sf_unit
    sf_unit = GET_FLASH_UNIT(current_global_addr);
    local_address = GET_LOCAL_ADDR(current_global_addr);

    // Write everything in buffer to the flash
    err_code = flash_write_bytes(RAM_buff, RAM_buff_size, sf_unit, local_address);
    if (err_code == NRFX_SUCCESS)
    {
        // Adjust the buffer size
        RAM_buff_size = 0;

        // Increase the global address pointer
        // This stays 256 because we write on a page-basis
        current_global_addr += 256;

        if (current_global_addr >= MAX_FLASH_SPACE)
        {
            NRFX_LOG_WARNING("Storage is full!");
        }
    }
    else
    {
        NRFX_LOG_ERROR("flash_write_bytes() returned: %d", err_code);
        return err_code;
    }
    return err_code;
}

/**
 * @brief  Read timestamp from flash
 * @param  data: buffer to store the timestamp from flash
 * @param  length: length of data to be recorded to buffer from flash
 * @param  global_address: global address, address location assuming all 4 flash as one unit
 * @return Error code.
 */
ret_code_t flash_read(char* data, size_t length, uint32_t global_address)
{
    uint32_t local_address;
    iVCCS_SF_array_unit sf_unit;
    ret_code_t err_code;

    // Calculate the Flash Unit to be initialized and its corresponding local address
    sf_unit = GET_FLASH_UNIT(global_address);
    local_address = GET_LOCAL_ADDR(global_address);

    // Configure the Flash Unit initialized
    err_code = flash_init(sf_unit);
    APP_ERROR_CHECK(err_code);

    // Read from Flash into buffer
    err_code = nrfx_qspi_read(data, length, local_address);
    APP_ERROR_CHECK(err_code);

    // Uninitialize the unit
    flash_uninit();

    NRFX_LOG_INFO("Read operation is done.");

    return err_code;
}

/**
 * @brief  Erase one sector of the flash unit starting at given address
 * @param  unit: Flash unit corresponds to the address
 * @param  addr: Start address for sector erase
 * @param  sectors: Number of sectors to be erased
 * @return Error code.
 */
ret_code_t flash_erase_sec(iVCCS_SF_array_unit unit, uint32_t addr, uint16_t sectors)
{
    ret_code_t err_code;

    // Check if address points to the beginning of a sector
    if (addr % (16*256) != 0)
    {
        NRFX_LOG_ERROR("Address <0x%08x> does not point to a beginning of a sector.", addr);
        return NRFX_ERROR_INVALID_ADDR;
    }

    // Check if the number of sectors does not exceed the flash size
    if(addr + (sectors * (16*256)) > FLASH_UNIT_SIZE)
    {
        NRFX_LOG_ERROR("Specified address <0x%08x> and number of sectors <%d> to erase exceed flash size.", addr, sectors);
        return NRFX_ERROR_INVALID_LENGTH;
    }

    // Initialize the flash unit
    err_code = flash_init(unit);
    APP_ERROR_CHECK(err_code);
    
    // Multiple sector erase procedure
    for (int i = 0; i < sectors; i++)
    {
        addr = addr + (i*16*256);
        err_code = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr);
        APP_ERROR_CHECK(err_code);

        // Wait for WIP bit
        WAIT_FOR_WIP();
    }

    NRFX_LOG_INFO("Sector erase is done.");

    // Reset global address if it's within the requested region of erase
    if (unit == GET_FLASH_UNIT(current_global_addr))
    {
        uint32_t local_addr = GET_LOCAL_ADDR(current_global_addr);
        if (local_addr >= addr && local_addr <= (addr + 16*256))
        {
            current_global_addr = addr + (FLASH_UNIT_SIZE * (unit - 1));
            RAM_buff_size = 0;
            NRFX_LOG_INFO("Global address is reset to 0x%08x.", current_global_addr);
        }
    }

    // Uninitialize the unit
    flash_uninit();

    return err_code;
}

/**
 * @brief  Erase the whole chip.
 * @param  unit: Flash unit to erase.
 * @return Error code.
 */
ret_code_t flash_chip_erase(iVCCS_SF_array_unit sf_unit)
{
    ret_code_t err_code;

    // Configure the Flash Unit initialized
    err_code = flash_init(sf_unit);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_qspi_chip_erase();
    APP_ERROR_CHECK(err_code);

    // Wait for WIP bit
    WAIT_FOR_WIP();

    NRFX_LOG_INFO("Flash unit %d erase is done.", sf_unit);
    
    // Reset global address if it's within the requested region of erase
    if (sf_unit == GET_FLASH_UNIT(current_global_addr))
    {
        current_global_addr = 0;
        RAM_buff_size = 0;
        NRFX_LOG_INFO("Global address is reset.");
    }

    // Uninitialize the sf_unit
    flash_uninit();

    return err_code;
}

/**
 * @brief  Perform chip erase on all flash units.
 * @return Error code.
 */
ret_code_t flash_all_erase(void)
{
    ret_code_t err_code;

    for (int i = 0; i < 4; i++)
    {
        // Configure the Flash Unit initialized
        err_code = flash_init(i);
        APP_ERROR_CHECK(err_code);

        err_code = nrfx_qspi_chip_erase();
        APP_ERROR_CHECK(err_code);

        // Wait for WIP bit
        WAIT_FOR_WIP();

        // Uninitialize the unit
        flash_uninit();
    }

    current_global_addr = 0;

    NRFX_LOG_INFO("All units erase is done.");

    return err_code;
}

/**
 * @brief  Return size of occupied data in flash in bytes
 * @return Data size.
 */
uint32_t flash_data_size()
{
    return current_global_addr;
}

/**
 * @brief  Return size of occupied data in flash-RAM buffer in bytes
 * @return RAM buffer size.
 */
uint16_t flash_buffer_size()
{
    return RAM_buff_size;
}

/**
 * @brief  Return current global address, points to the next write location
 *         It should be a multiple of 256 bytes.
 * @return Current global address.
 */
uint32_t flash_get_global_ptr()
{
    return current_global_addr;
}

/**
 * @brief  Finds the first unoccupied address in the flash array.
 *
 * The function reads the first two bytes of each page and examines if they are 0xFF
 * to determine whether the page is occupied or not, since the write procedure is
 * a page-basis.
 * @return Global address.
 */
uint32_t flash_seek()
{
    uint32_t local_address;
    uint32_t global_address;
    iVCCS_SF_array_unit sf_unit;
    ret_code_t err_code;
    uint8_t data[4];   // We read 4 bytes because it looks like we can only read in multiples of 4, propably because of Quad-SPI.
    bool address_found = false;

    for (sf_unit = iVCCS_SF1; sf_unit <= iVCCS_SF4; sf_unit++)
    {
        err_code = flash_init(sf_unit);
        APP_ERROR_CHECK(err_code);
        local_address = 0x00;

        while(!address_found && (local_address < FLASH_UNIT_SIZE))
        {
            // Read from Flash into buffer
            err_code = nrfx_qspi_read(data, 4, local_address);
            APP_ERROR_CHECK(err_code);
            
            NRFX_LOG_HEXDUMP_DEBUG(data, 4);

            if ((data[0] == 0xFF || data[0] == 0x88) && (data[1] == 0xFF || data[1] == 0x88))
                address_found = true;
            else
                local_address += 256;
        }

        // Uninitialize the unit
        flash_uninit();

        if (address_found) break;
    }

    if (address_found)
    {
        global_address = local_address + (FLASH_UNIT_SIZE * (sf_unit - 1));
        current_global_addr = global_address;
        return global_address;
    }

    return 0x00;
}