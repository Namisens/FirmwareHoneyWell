/*
 * MX25R1635F.h
 *
 *  Created on: 14.05.2019
 *      Author: Nicolas.Ennaji
 */

#ifndef MX25R1635F_H_
#define MX25R1635F_H_

/**
 * @def MX25R1635F_INVALIDADDRESS
 * @brief Flash invalid address
 */
#define MX25R1635F_INVALIDADDRESS 0x1000000UL

/**
 * @def MX25R1635F_BLOCK_COUNT
 * @brief Number of blocks inside the flash memory
 */
#define MX25R1635F_BLOCK_COUNT 32

/**
 * @def MX25R1635F_BLOCK_SIZE
 * @brief Size of one block (in bytes)
 */
#define MX25R1635F_BLOCK_SIZE 0x10000UL

/**
 * @def MX25R1635F_SECTORS_PER_BLOCK
 * @brief Number of sectors per block
 */
#define MX25R1635F_SECTORS_PER_BLOCK 16

/**
 * @def MX25R1635F_SECTOR_SIZE
 * @brief Size of one sector (in bytes)
 */
#define MX25R1635F_SECTOR_SIZE 0x1000UL

/**
 * @def MX25R1635F_PAGES_PER_SECTOR
 * @brief Number of pages per sector
 */
#define MX25R1635F_PAGES_PER_SECTOR 16

/**
 * @def MX25R1635F_PAGE_SIZE
 * @brief Size of one page (in bytes)
 */
#define MX25R1635F_PAGE_SIZE 0x100UL

/**
 * @brief Initialise the communication with the MX25R1635F flash memory
 *
 * Initialise the communication with the MX25R1635F flash memory
 */
void mx25r1635finitialise();

/**
 * @brief Check if the buffer is empty
 *
 * Check if the buffer is empty
 *
 * @param [in] buffer   Buffer to check
 * @param [in] index    Index where to start
 *
 * @retval 0 Buffer not empty
 * @retval 1 Buffer empty
 */
uint8_t mx25r1635f_is_buffer_empty(uint8_t *buffer, uint16_t index);

/**
 * @brief Read the status register
 *
 * Read the status register
 * @param [out] content
 */
uint16_t mx25r1635f_read_statusregister(uint8_t *content);

uint16_t mx25r1635f_read_id(uint8_t* id);

/**
 * @brief Check if a write operation is in progress
 *
 * Check if a write operation is in progress
 *
 * @retval 0 No write in progress
 * @retval 1 Write in progress
 */
uint8_t mx25r1635f_is_writeinprogress();

/**
 * @brief Wait until a write is in progress
 *
 * Wait until a write is in progress
 */
void mx25r1635f_wait_writeinprogress();

/**
 * @brief Enable write access on the module
 *
 * Enable write access on the module
 */
void mx25r1635f_write_enable();

/**
 * @brief Read data from the flash memory
 *
 * Read data from the flash memory
 *
 * @param [in]  address Address where to read (24 bit used)
 * @param [out] buffer  Pointer on a buffer that will be filled
 * @param [in]  size    Size of the data to be read (in byte)
 */
uint16_t mx25r1635f_read_data(uint32_t address,
        uint8_t *buffer,
        uint16_t size);

/**
 * @brief Write data to the flash memory
 *
 * Write data to the flash memory\n
 * Before calling this function, you should call mx25r1635fwriteenable
 *
 * @param [in] address  Address where to write (24 bits used)
 * @param [in] buffer   Pointer on the buffer to be written
 * @param [in] size     Size of the data to be written (in byte)
 */
uint16_t mx25r1635f_write_data(uint32_t address,
        uint8_t *buffer,
        uint16_t size);

/**
 * @brief Write data to the flash memory and wait
 *
 * Write data to the flash memory and wait.\n
 * This function also enables the write process
 *
 * @param [in] address  Address where to write (24 bits used)
 * @param [in] buffer   Pointer on the buffer to be written
 * @param [in] size     Size of the data to be written (in byte)
 */
uint16_t mx25r1635f_write_data_and_wait(uint32_t address,
        uint8_t *buffer,
        uint16_t size);

/**
 * @brief Erase the selected sector
 *
 * Erase the selected sector
 *
 * @param [in] address Address of the sector (24 bits used)
 */
void mx25r1635f_sector_erase(uint32_t address);

/**
 * @brief Erase the selected block
 *
 * Erase the selected block
 *
 * @param [in] address Address of the sector (24 bits used)
 */
void mx25r1635f_block_erase(uint32_t address);

/**
 * @brief Erase the entire flash memory array
 *
 * Erase the entire flash memory array
 */
void mx25r1635f_chip_erase();

/**
 * @brief Get the first free available slot where it is possible to write
 *
 * Get the first free available slot where it is possible to write
 *
 * @param [in] startblock       Block at which the first free slot search should
 * begin
 * @param [in] blocksize        How much block should be searched through
 * @param [in] elementperpage   How much elements fit in one page
 * @param [in] elementsize      Size of one element stored at that place
 * @param [in] pageoffset       Offset at the beginning of the page with dummy
 * data
 *
 * @return Address of the first free slot on 4 bytes
 */
uint32_t mx25r1635fgetfirstfreeslot(uint16_t startblock,
        uint16_t blocksize,
        uint16_t elementperpage,
        uint16_t elementsize,
        uint16_t pageoffset);

/**
 * @brief Erase sector by sector
 *
 * Erase sector by sector
 *
 * @param [in] startblock       Block at which the erasing should occur
 * @param [in] blocksize        How much blocks are concerned
 * @param [in] stopwhenempty    Should the process stop when next sector is empty
 */
void mx25r1635ferasesectorbysector(uint16_t startblock,
        uint16_t blocksize, uint8_t stopwhenempty);

/**
 * @brief Compute an address given Flash structure indexes.
 *
 * @param [in] block        Block index inside the Flash
 * @param [in] sector       Sector index inside the Flash
 * @param [in] page         Page index inside the Flash
 * @param [in] element      Index of the element within the page
 * @param [in] elementsize  Size of one element
 * @param [in] pageoffset   Offset within the page, representing non used data
 * at the beginning of the page
 *
 * @return Computed address on 4 bytes
 */
uint32_t computeaddress(uint16_t block,
        uint16_t sector,
        uint16_t page,
        uint16_t element,
        uint16_t elementsize,
        uint16_t pageoffset);

/**
 * @brief Compute an address given a start address and an index of elements
 * stored at that address.
 *
 * @param [in] index             Increment of the whished element address
 * @param [in] startaddress      Start address at which the address calculation
 * should begin
 * @param [in] elementperpage    Number of elements within the page
 * @param [in] elementsize       Size of the element to increment
 * @param [in] pageoffset        Offset within the page, representing non used data
 * at the beginning of the page
 *
 * @return Computed address on 4 bytes
 */
uint32_t s25fl216readaddresscalc(uint32_t index,
        uint32_t startaddress,
        uint32_t elementperpage,
        uint32_t elementsize,
        uint32_t pageoffset);


#endif /* MX25R1635F_H_ */
