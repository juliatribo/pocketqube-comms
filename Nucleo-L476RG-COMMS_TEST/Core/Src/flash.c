/*!
 * \file      flash.c
 *
 * \brief     It contains all the functions to read from / write in the flash memory
 *
 *
 * \created on: 15/05/2022
 *
 * \author    Daniel Herencia
 */

#include "flash.h"
#include "stm32l4xx_hal.h"
#include "string.h"
#include "stdio.h"

/**************************************************************************************
 *                                                                                    *
 * Function:  Get_Page 	                                                    		  *
 * --------------------                                                               *
 * Gets the page of a given address			        								  *
 *                                                                                    *
 *  Address: Specific address of a read/write function                                *
 *                                                                                    *
 *  returns: page in which the address is contained			                          *
 *                                                                                    *
 **************************************************************************************/
static uint32_t Get_Page(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr <(FLASH_BASE + FLASH_BANK_SIZE))
  {
   /* Bank 1 */
    page = (Addr-FLASH_BASE)/FLASH_PAGE_SIZE;
  }
  else
  {
   /* Bank 2 */
    page = (Addr-(FLASH_BASE + FLASH_BANK_SIZE))/FLASH_PAGE_SIZE;
  }

  return page;
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Get_Bank                        	                             		  *
 * --------------------                                                               *
 * brief Gets the bank of a given address       									  *											  *
 *                                                                                    *
 *  Address: Specific address of a read/write function                                *
 *                                                                                    *
 *  returns: The bank of a given address					                          *
 *                                                                                    *
 **************************************************************************************/

/**
  * @brief Gets the bank of a given address
  * @param Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t Get_Bank(uint32_t Addr)
{
  if (Addr <(FLASH_BASE + FLASH_BANK_SIZE))
	  return FLASH_BANK_1;
  else
	  return FLASH_BANK_2;
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Flash_Write_Data                                                 		  *
 * --------------------                                                               *
 * Writes in the flash memory														  *
 *                                                                                    *
 *  Address: first address to be written		    		                          *
 *	Data: information to be stored in the FLASH memory								  *
 *	numberofbytes: Data size in Bytes					    						  *
 *															                          *
 *  returns: Nothing or error in case it fails			                              *
 *                                                                                    *
 **************************************************************************************/
uint32_t Flash_Write_Data(uint32_t Address, uint64_t *Data_write, uint16_t numberofbytes) {

	// MY FLASH_WRITE_DATA
	uint32_t init_page = Get_Page( Address );
	uint32_t finish_page = Get_Page( Address + 8*numberofbytes );
	// Unlock the Flash to enable the flash control register access
	HAL_FLASH_Unlock();

	//pensar que pasa cuando los datos a escribir ocupan varias paginas
	//la segunda iteracion del while hay que revisarla, porque ahora escribe lo mismo que la primera

	while(init_page <= finish_page){
		//Safe last page
		uint64_t Data_ans[PAGESIZE/8];
		uint32_t page_start_add;
		if (Address <(FLASH_BASE + FLASH_BANK_SIZE))
			page_start_add = FLASH_BASE + PAGESIZE*init_page;
		else
			page_start_add = FLASH_BASE + FLASH_BANK_SIZE + PAGESIZE*init_page;

		Flash_Read_Data(page_start_add,&Data_ans,PAGESIZE/8);

		//Erase Page
		static FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t PAGEError;
		EraseInitStruct.Banks = Get_Bank( Address );//Bank where the erase address is located
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;//Erase by page
		EraseInitStruct.Page = Get_Page( Address );//Get page position
		EraseInitStruct.NbPages = 1;//Erase 1 page
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
			return HAL_FLASH_GetError();
		}

		uint32_t difference = (Address - page_start_add) / 8;


		//Write new data
		uint8_t bytes_write = numberofbytes;
		uint32_t write_address = Address;
		uint8_t position_wr = 0;

		while(bytes_write > 0){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_address, Data_write[position_wr]);
			write_address += 8;
			bytes_write--;
			position_wr++;
		}

		//Write old data

		uint16_t bytes_write2 = PAGESIZE / 8;
		write_address = page_start_add;
		uint16_t position_wr2 = 0;
		while(bytes_write2 > 0){
			if (write_address != Address && Data_ans[position_wr2] != 0xFFFFFFFFFFFFFFFF){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_address, Data_ans[position_wr2]);
			}
			write_address += 8;
			bytes_write2--;
			position_wr2++;
		}

		init_page++;
	}
	// Lock the Flash to disable the flash control register access (recommended
	 //to protect the FLASH memory against possible unwanted operation)
	HAL_FLASH_Lock();

	return 0;

	/*
	//MIREIA'S FUNCTION
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar = 0;

	// Unlock the Flash to enable the flash control register access
	HAL_FLASH_Unlock();

	// Erase the user Flash area

	uint32_t StartPage = Get_Page(Address);
	uint32_t EndPageAdress = Address + numberofbytes * 8;
	uint32_t EndPage = Get_Page(EndPageAdress);

	// Fill EraseInit structure
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = StartPage;
	EraseInitStruct.Banks = Get_Bank(Address);
	EraseInitStruct.NbPages = (EndPage - StartPage) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {

		return HAL_FLASH_GetError();
	}
	while (sofar < numberofbytes) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data_write[sofar]) == HAL_OK) {
			Address += 8; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		} else {
			// Error occurred while writing data in Flash memory
			return HAL_FLASH_GetError();
		}
	}

	// Lock the Flash to disable the flash control register access (recommended
	 //to protect the FLASH memory against possible unwanted operation)
	HAL_FLASH_Lock();

	return 0;
	*/
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Write_Flash                                                		 	  *
 * --------------------                                                               *
 * It's the function that must be called when writing in the Flash memory.			  *
 * Depending on the address, it writes 1 time or 3 times (Redundancy)				  *
 *                                                                                    *
 *  StartSectorAddress: first address to be written		                              *
 *	Data: information to be stored in the FLASH/EEPROM memory						  *
 *	numberofbytes: Data size in Bytes					    						  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void Write_Flash(uint32_t StartSectorAddress, uint64_t *Data, uint16_t numberofbytes) {
	if (StartSectorAddress >= 0x08000000 && StartSectorAddress <= 0x0800BFFF) { //addresses with redundancy
	// The addresses are separated 0x4000 positions
		Flash_Write_Data(StartSectorAddress, Data, numberofbytes);
		//Flash_Write_Data(StartSectorAddress + 0x4000, Data, numberofbytes);
		//Flash_Write_Data(StartSectorAddress + 0x8000, Data, numberofbytes);
		//Flash_Write_Data(StartSectorAddress + 0x0040, Data, numberofbytes);
		//Flash_Write_Data(StartSectorAddress + 0x0080, Data, numberofbytes);
	} else {
		Flash_Write_Data(StartSectorAddress, Data, numberofbytes);
	}
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Flash_Read_Data                                                 		  *
 * --------------------                                                               *
 * Reads from the flash memory														  *
 *                                                                                    *
 *  Address: first address to be read					                              *
 *	Data_read: Where the data read from memory will be stored						  *
 *	numberofbytes: Reading data size in Bytes					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void Flash_Read_Data(uint32_t Address, uint64_t *Data_read, uint16_t numberofbytes) {
	uint16_t bytes_read = numberofbytes;
	uint32_t read_address = Address;
	uint8_t position_rd = 0;
	/*
	uint32_t init_page = Get_Page( Address );
	uint16_t pagenumber = Get_Page( Address );
	uint32_t page_start_add;
	if (Address <(FLASH_BASE + FLASH_BANK_SIZE))
		page_start_add = FLASH_BASE + PAGESIZE*init_page;
	else
		page_start_add = FLASH_BASE + FLASH_BANK_SIZE + PAGESIZE*init_page;
*/
	/*uint8_t number_of_pages = 1;
	while(Address+numberofbytes > page_start_add + PAGESIZE){
		number_of_pages++;
		number_of_bytes = number_of_bytes - PAGESIZE;
	}
	while(number_of_pages>0){*/
	while (bytes_read > 0) {
		  Data_read[position_rd] = *(__IO uint64_t*) read_address;
		  read_address += 8;
		  bytes_read--;
		  position_rd++;
		  /*if(bytes_read%10==0){
			  uint8_t variable = 0;
		  }
		  if(read_address + 8 >= page_start_add+PAGESIZE){
			  read_address += 8;
		  }*/
	}
		//number_of_pages--;
	//}
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Check_Redundancy                                                 		  *
 * --------------------                                                               *
 * Reads the data from the 3 addresses where it is stored and chooses the value		  *
 * that coincides at least in 2 of the 3 addresses (in case one gets corrupted)		  *
 * All the addresses of variables with Redundancy follow the same pattern: each		  *
 * address is separated 0x4000 positions in memory									  *
 *                                                                                    *
 *  Address: first address to be read		                              			  *
 *	RxDef: Buffer to store the lecture that coincides at least 2 times				  *
 *	numberofbytes: Data size in bytes												  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void Check_Redundancy(uint32_t Address, uint64_t *RxDef, uint16_t numberofbytes) {
	uint64_t lect1[numberofbytes], lect2[numberofbytes], lect3[numberofbytes];
	Flash_Read_Data(Address, lect1, numberofbytes);
	Flash_Read_Data(Address + 0x4000, lect2, numberofbytes);
	Flash_Read_Data(Address + 0x8000, lect3, numberofbytes);
	//Flash_Read_Data(Address + 0x0040, lect2, numberofbytes);
	//Flash_Read_Data(Address + 0x0080, lect3, numberofbytes);

	bool coincidence12 = true, coincidence13 = true, coincidence23 = true;
	for (int i = 0; i < numberofbytes; i++) {
		if (lect1[i] != lect2[i])
			coincidence12 = false;
		if (lect1[i] != lect3[i])
			coincidence13 = false;
		if (lect2[i] != lect3[i])
			coincidence23 = false;
	}
	if (coincidence12 || coincidence13) {
		Flash_Read_Data(Address, RxDef, numberofbytes);
	} else if (coincidence23) {
		Flash_Read_Data(Address + 0x4000, RxDef, numberofbytes);
	}
	else {
		*RxDef = lect1; /*PREGUNTAR QUÈ FER QUAN NO COINCIDEIX CAP LECTURA (POC PROBABLE)*/
	}
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Read_Flash	                                                 		  *
 * --------------------                                                               *
 * It's the function that must be called when reading from the Flash memory.		  *
 * Depending on the address, it reads from 1 or 3 addresses (Redundancy)			  *
 *                                                                                    *
 *  StartSectorAddress: starting address to read		                              *
 *	RxBuf: Where the data read from memory will be stored							  *
 *	numberofbytes: Reading data size in Bytes					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void Read_Flash(uint32_t StartSectorAddress, uint64_t *RxBuf, uint16_t numberofbytes) {
	Flash_Read_Data(StartSectorAddress, RxBuf, numberofbytes);
/*
	if (StartSectorAddress >= 0x08000000 && StartSectorAddress <= 0x0800BFFF) { //addresses with redundancy
		Check_Redundancy(StartSectorAddress, RxBuf, numberofbytes);
	} else {
		Flash_Read_Data(StartSectorAddress, RxBuf, numberofbytes);
	}*/
}
