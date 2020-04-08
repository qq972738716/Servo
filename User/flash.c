#include "flash.h"
static FLASH_EraseInitTypeDef EraseInitStruct = {
	.TypeErase = FLASH_TYPEERASE_PAGES,
	.PageAddress = 0x08003C00,
	.NbPages = 1
};
BOOL Flash_write(uint32_t address,uint16_t* pbuff,uint16_t buffsize)
{
			HAL_FLASH_Unlock();
			uint32_t PageError = 0;
			if (HAL_FLASHEx_Erase(&EraseInitStruct,&PageError) != HAL_OK)
			{
				return FALSE;
			}
			for(uint16_t i = 0; i< buffsize;i++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,address+2*i, *(pbuff+i));
			}
			 HAL_FLASH_Lock();
			return TRUE;
}
BOOL Flash_Read(uint32_t address,uint16_t* pbuff,uint16_t buffsize)
{
				for(uint16_t i = 0; i< buffsize;i++)
			{
				*(pbuff+i) = *(__IO uint16_t *)(address+2*i);
			}
		return TRUE;
}
