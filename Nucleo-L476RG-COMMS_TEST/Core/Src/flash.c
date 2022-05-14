/*!
 * \file      flash.c
 *
 * \brief     It contains all the functions to read from / write in the flash memory
 *
 *
 * \created on: 15/11/2021
 *
 * \author    Pol Simon
 *
 * \author    David Reiss
 */

#include "flash.h"
#include "stm32l4xx_hal.h"
#include "string.h"
#include "stdio.h"

/**************************************************************************************
 *                                                                                    *
 * Function:  GetSector                                                     		  *
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
 * Function:  FirstAddress                                                     		  *
 * --------------------                                                               *
 * defines the first address of a sector according to the reference manual	          *											  *
 *                                                                                    *
 *  sector: Specific sector of a read/write function                                  *
 *                                                                                    *
 *  returns: The first address corresponding to the sector	                          *
 *                                                                                    *
 **************************************************************************************/



/**
  * @brief Gets the bank of a given address
  * @param Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t Get_Bank(uint32_t Addr)
{
  uint32_t bank = 0;

//if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
//{
///* No Bank swap */
//if (Addr <(FLASH_BASE + FLASH_BANK_SIZE))
//{
//bank = FLASH_BANK_1;
//}
//else
//{
//bank = FLASH_BANK_2;
//}
//}
//else
//{
///* Bank swap */
//if (Addr <(FLASH_BASE + FLASH_BANK_SIZE))
//{
//bank = FLASH_BANK_2;
//}
//else
//{
//bank = FLASH_BANK_1;
//}
//}

//return bank;
    return FLASH_BANK_1;//L431 only has FLASH_BANK_1 REVISE THIS LINE!!!!!!!!!!!!!!!!!!
}

/**************************************************************************************
 *                                                                                    *
 * Function:  Flash_Write_Data                                                 		  *
 * --------------------                                                               *
 * Writes in the flash memory														  *
 *                                                                                    *
 *  StartSectorAddress: first address to be written		                              *
 *	Data: information to be stored in the FLASH/EEPROM memory						  *
 *	numberofbytes: Data size in Bytes					    						  *
 *															                          *
 *  returns: Nothing or error in case it fails			                              *
 *                                                                                    *
 **************************************************************************************/
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint64_t *Data, uint16_t numberofbytes) {

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar = 0;


	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area */

	/* Get the number of sector to erase from 1st sector */
	uint32_t StartSector = 0x08008000;

	uint32_t FirstAddr = 0x08008000;
	uint32_t Addr = FirstAddr;
	uint32_t N_ADDR = 50;
	uint32_t LastAddr = FirstAddr+N_ADDR;
	uint64_t DataSave[PAGESIZE];
	uint8_t i = 0,j=0;
	Data = *(__IO uint64_t*)Addr;
	/*
	while(Addr < LastAddr){
		// Verify if Addr is the Address where we want to write
		if (Addr == StartSectorAddress){
			// We add the data array to DataSave
			while(j<numberofbytes){
				DataSave[i] = Data[j];
				i++;
				j++;
				Addr++;
			}
		}else{
			DataSave[i] = *(__IO uint64_t*)Addr;
			i++;
			Addr = Addr+1;
		}
	}
	*/
	 /* Clear OPTVERR bit set on virgin samples */
	 //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PROGERR | FLASH_FLAG_BSY);
	/* Fill EraseInit structure*/
	 /* Get the number of sector to erase from 1st sector*/
	  EraseInitStruct.Banks = Get_Bank(StartSectorAddress);//Get the bank where the erase address is located
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;//Erase by page
	  EraseInitStruct.Page = Get_Page(StartSectorAddress);//Get page position
	  EraseInitStruct.NbPages = 1;//Erase 1 page

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		return HAL_FLASH_GetError();

	}

	/* Program the user Flash area word by word
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//	while (sofar < numberofbytes) {
//		uint8_t dd = Data[sofar];
//		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, StartSectorAddress,
//				Data[sofar]) == HAL_OK) {
//			StartSectorAddress += 1; // use StartPageAddress += 2 for half word and 8 for double word
//			sofar++;
//		} else {
//			/* Error occurred while writing data in Flash memory*/
//			return HAL_FLASH_GetError();
//		}
//	}

	while (sofar < N_ADDR) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FirstAddr,
				DataSave[sofar]) == HAL_OK) {
			FirstAddr += 8; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
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
 *  StartSectorAddress: first address to be read		                              *
 *	RxBuf: Where the data read from memory will be stored							  *
 *	numberofbytes: Reading data size in Bytes					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void Flash_Read_Data(uint32_t StartSectorAddress, uint64_t *RxBuf, uint16_t numberofbytes) {
	while (1) {
		*RxBuf = *(__IO uint64_t*) StartSectorAddress;
		StartSectorAddress += 1;	//perque += 1 ????
		RxBuf++;
		numberofbytes--;
		if (numberofbytes == 0)
			break;
	}
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
