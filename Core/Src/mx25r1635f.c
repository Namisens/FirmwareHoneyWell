/*
 * mx25r1635f.c
 *
 *  Created on: 14.05.2019
 *      Author: Nicolas.Ennaji
 */


#include "stm32g0xx.h"
#include "main.h" //Check if this is necessary
#include "mx25r1635f.h"
#include "spi.h"


/**
 * @def MX25R1635F_PAGE_PROGRAM_CMD
 * @brief Flash command to program a page.
 */
#define MX25R1635F_PAGE_PROGRAM_CMD 0x02

/**
 * @def MX25R1635F_READ_DATA_CMD
 * @brief Flash command to read data.
 */
#define MX25R1635F_READ_DATA_CMD 0x03

/**
 * @def MX25R1635F_WRITE_DISABLE_CMD
 * @brief Flash command to disable writing.
 */
#define MX25R1635F_WRITE_DISABLE_CMD 0x04

/**
 * @def MX25R1635F_READ_STATUS_REGISTER_CMD
 * @brief Flash command to read status register.
 */
#define MX25R1635F_READ_STATUS_REGISTER_CMD 0x05

/**
 * @def MX25R1635F_WRITE_ENABLE_CMD
 * @brief Flash command to enable writing.
 */
#define MX25R1635F_WRITE_ENABLE_CMD 0x06

/**
 * @def MX25R1635F_READ_JECED_ID_CMD
 * @brief Flash command to read JECED identifier.
 */
#define MX25R1635F_READ_JECED_ID_CMD 0x9F

/**
 * @def MX25R1635F_SECTOR_ERASE_CMD
 * @brief Flash command to erase a sector.
 */
#define MX25R1635F_SECTOR_ERASE_CMD 0x20

/**
 * @def MX25R1635F_CHIP_ERASE_CMD
 * @brief Flash command to erase the complete chip.
 */
#define MX25R1635F_CHIP_ERASE_CMD 0xC7

/**
 * @def MX25R1635F_BLOCK_ERASE_CMD
 * @brief Flash command to erase a block.
 */
#define MX25R1635F_BLOCK_ERASE_CMD 0xD8

/**
 * @brief Do some delay for the communication
 *
 * Do some delay for the communication
 */
static inline void mx25R1635f_delay()
{
    int i = 0;
    for (i = 0; i < 40; ++i) asm("NOP");
}

/**
 * @brief Send the address
 *
 * Send the address
 *
 * @param address Address to be sent
 */
static void mx25r1635f_send_address(uint32_t address)
{
	uint8_t ad;
	ad = ((address >> 16) & 0xFF);
	HAL_SPI_Transmit(&hspi1, &ad, 1, 100);
	ad = ((address >> 8) & 0xFF);
	HAL_SPI_Transmit(&hspi1, &ad, 1, 100);
	ad = (address & 0xFF);
	HAL_SPI_Transmit(&hspi1, &ad, 1, 100);
}

/**
 * @brief Send a command
 *
 * Send a command
 *
 * @param cmd Command to be sent
 */
static void mx25r1635f_send_command(uint8_t cmd)
{
	/*HAL_SPI_Transmit(&cmd, 1);*/
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
}


uint8_t mx25r1635f_is_buffer_empty(uint8_t *buffer, uint16_t index)
{
    if ((buffer[index] == 0xFF)
				&& (buffer[index+1] == 0xFF)
				&& (buffer[index+2] == 0xFF)
				&& (buffer[index+3] == 0xFF))
        return (1);
    return (0);
}

uint16_t mx25r1635f_read_id(uint8_t* id)
{
	uint16_t ret;

	//spi_select();
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

	// Send command
	mx25r1635f_send_command(MX25R1635F_READ_JECED_ID_CMD);

	// Get answer with 100
	ret=HAL_SPI_Receive(&hspi1, id, 3, 100);

	//spi_deselect();
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

	return (ret);
}

uint16_t mx25r1635f_read_statusregister(uint8_t *content)
{
	uint16_t ret;

    // default
    *content = 0;

    //spi_select();
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_READ_STATUS_REGISTER_CMD);

    // Get answer
    //ret = spi_read(content, 1);
    ret=HAL_SPI_Receive(&hspi1, content, 1, 100);


    //spi_deselect();
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    return (ret);
}

uint8_t mx25r1635f_is_writeinprogress()
{
    uint8_t statusregister = 0;
    mx25r1635f_read_statusregister(&statusregister);
    if ((statusregister & 1) == 0) return (0);
    else return (1);
}

void mx25r1635f_wait_writeinprogress(void)
{
    for(;;)
	{
		if (mx25r1635f_is_writeinprogress() == 0) break;
		else HAL_Delay(10);
	}
}

void mx25r1635f_write_enable()
{
    //spi_select();
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_WRITE_ENABLE_CMD);

    //spi_deselect();
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);
}

void mx25r1635f_write_disable()
{
    //spi_select();   // select
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_WRITE_DISABLE_CMD);

    //spi_deselect();   // release
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);
}

uint16_t mx25r1635f_read_data(uint32_t address,
        uint8_t *buffer,
        uint16_t size)
{
	uint16_t ret;

    //spi_select();   // select
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_READ_DATA_CMD);

    // Send address
    mx25r1635f_send_address(address);

    // Get answer
    //ret = spi_read(buffer, size);
    ret= HAL_SPI_Receive(&hspi1, buffer, size, 100); //

    //spi_deselect();   // release
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    return (ret);
}

uint16_t mx25r1635f_write_data(uint32_t address,
        uint8_t *buffer,
        uint16_t size)
{
	uint16_t ret;

    //spi_select();   // select
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_PAGE_PROGRAM_CMD);

    // Send address
    mx25r1635f_send_address(address);

    // Send data
    ret = HAL_SPI_Transmit(&hspi1, buffer, size, 100);

    //spi_deselect();   // release
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    return (ret);
}

uint16_t mx25r1635f_write_data_and_wait(uint32_t address,
        uint8_t *buffer,
        uint16_t size)
{
    uint16_t written;
    uint8_t writable = 1;

// First check that the area willing to be written is empty
//TODO change for not dynamically created array
//    uint16_t i = 0;
//    uint8_t bufferClone[size];
//
//
//    mx25r1635f_read_data(address, bufferClone, size);
//    for (i = 0; i < size; ++i)
//    {
//        if (bufferClone[i] != 0xFF)
//        {
//            writable = 0;
//            break;
//        }
//    }

    if (!writable)
        return (0);

    // Then write
    mx25r1635f_write_enable();
    written = mx25r1635f_write_data(address, buffer, size);
    mx25r1635f_wait_writeinprogress();

    return (written);
}


void mx25r1635f_block_erase(uint32_t address)
{
	//spi_select();   // select
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

	// send the command
	mx25r1635f_send_command(MX25R1635F_BLOCK_ERASE_CMD);

    // Send address
    mx25r1635f_send_address(address);

	//spi_deselect();   // release
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);
}

void mx25r1635f_sector_erase(uint32_t address)
{
    //spi_select();   // select
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_SECTOR_ERASE_CMD);

    // Send address
    mx25r1635f_send_address(address);

    //spi_deselect();   // release
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);
}

void mx25r1635f_chip_erase()
{
    //spi_select();   // select
	HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);

    // Send command
    mx25r1635f_send_command(MX25R1635F_CHIP_ERASE_CMD);

    //spi_deselect();   // release
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);
}

uint32_t computeaddress (uint16_t block,
        uint16_t sector,
        uint16_t page,
        uint16_t element,
        uint16_t elementsize,
        uint16_t pageoffset)
{
    uint32_t result = 0;
    uint32_t tmp = (uint32_t)block;
	uint32_t tmp1 = (uint32_t)sector;
	uint32_t tmp2 = (uint32_t)page;
	uint32_t tmp3 = (uint32_t)element;
    uint32_t tmp4 = (uint32_t)elementsize;

    result = (tmp * MX25R1635F_BLOCK_SIZE)
            + (tmp1 * MX25R1635F_SECTOR_SIZE)
            + (tmp2 * MX25R1635F_PAGE_SIZE);

    if (pageoffset != 0) result += pageoffset;
    if (element != 0) result += (tmp3 * tmp4);

    return (result);
}

uint32_t mx25r1635fgetfirstfreeslot(uint16_t startblock,
        uint16_t blocksize,
        uint16_t elementperpage,
        uint16_t elementsize,
        uint16_t pageoffset)
{
    uint16_t block = 0;
	uint16_t sector = 0;
	uint16_t page = 0;
	uint16_t element = 0;
	uint8_t toread[4] = {0};
	uint32_t address = 0;

	/* loop through the blocks (64kB) */
	for (block = startblock; block < (startblock + blocksize); ++block)
	{
        address = computeaddress(block, 0, 0, 0, elementsize, 0);

		/* read the header of the block */
		mx25r1635f_read_data(address, toread, 4);

        if (mx25r1635f_is_buffer_empty(toread, 0) != 0) break;
	}

	/* look the previous one */
	if (block > startblock) block = block - 1;

	/* loop through the sectors of the block */
	for (sector = 0; sector < MX25R1635F_SECTORS_PER_BLOCK; ++sector)
	{
        address = computeaddress(block, sector, 0, 0, elementsize, 0);

		/* read the header of the sector */
		mx25r1635f_read_data(address, toread, 4);

		if (mx25r1635f_is_buffer_empty(toread, 0) != 0) break;
	}

	/* look the previous one */
	if (sector > 0) sector = sector - 1;

	/* loop through the pages of the sector */
	for (page = 0; page < MX25R1635F_PAGES_PER_SECTOR; ++page)
	{
        address = computeaddress(block, sector, page, 0, elementsize, 0);

		/* read the header of the sector */
		mx25r1635f_read_data(address, toread, 4);

		if (mx25r1635f_is_buffer_empty(toread, 0) != 0) break;
	}

	/* look the previous one */
	if (page > 0) page = page - 1;

	/* loop through the elements of the page */
	for (element = 0; element < elementperpage; ++element)
	{
        address = computeaddress(block, sector, page, element, elementsize, pageoffset);

		/* read the header of the sector */
		mx25r1635f_read_data(address, toread, 4);

        if (mx25r1635f_is_buffer_empty(toread, 0) != 0)
        {
            if (element == 0)
                return computeaddress(block, sector, page, element, elementsize, 0);
            return (address);
        }
	}

	/* cannot find a free space */
	if (element == elementperpage)
	{
		if (page == (MX25R1635F_PAGES_PER_SECTOR - 1))
		{
			if (sector == (MX25R1635F_SECTORS_PER_BLOCK - 1))
			{
				if (block == (startblock + blocksize - 1))
				{
					/* full, write address stays invalid */
					return (MX25R1635F_INVALIDADDRESS);
				}
				else
				{
					element = 0;
					page = 0;
					sector = 0;
					block = block + 1;
				}
			}
			else
			{
				element = 0;
				page = 0;
				sector = sector + 1;
			}
		}
		else
		{
			element = 0;
			page = page + 1;
		}

        return (computeaddress(block, sector, page, element, elementsize, 0));
	}

    /* should never happen */
    return (MX25R1635F_INVALIDADDRESS);
}

void mx25r1635ferasesectorbysector(uint16_t startblock,
        uint16_t blocksize, uint8_t stopwhenempty)
{
    uint16_t block = 0;

	/* loop through the blocks (64kB) */
	for (block = startblock; block < (startblock + blocksize); ++block)
	{
		uint16_t sector = 0;

		/* loop through the sectors of the block */
		for (sector = 0; sector < MX25R1635F_SECTORS_PER_BLOCK; ++sector)
		{
			uint8_t toread[4] = {0};
			uint32_t address = computeaddress(block, sector, 0, 0, 0, 0);

			/* read the header of the sector */
			mx25r1635f_read_data(address, toread, 4);

            if ((toread[0] == 0xFF) && (stopwhenempty == 1)) return;
			else
			{
				/* erase the sector */
				mx25r1635f_write_enable();
				mx25r1635f_sector_erase(address);
				mx25r1635f_wait_writeinprogress();
			}
		}
	}
}

uint32_t mx25r1635freadaddresscalc(uint32_t index, uint32_t startaddress, uint32_t elementperpage, uint32_t elementsize, uint32_t pageoffset)
{
	uint32_t readat = startaddress * MX25R1635F_BLOCK_SIZE; // 0x120000
	uint32_t increment = (index / elementperpage) * MX25R1635F_PAGE_SIZE; // Addr page
	uint32_t pageindex = index % elementperpage;// position ds page

	readat += increment;

	if (pageindex != 0)
	{
		increment = pageoffset + (pageindex * elementsize); // OPENINGPERIOD_PAGE_OFFSET  OPENINGPERIOD_ELEMENT_SIZE
		readat += increment;
	}
	else
		readat += pageoffset;

	return (readat);
}
